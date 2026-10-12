#!/usr/bin/env bash
#
# robot_stack 监控脚本
#
# 功能：
#   1. 加载 ROS 2 / 工作空间环境
#   2. 等待 qi_deploy.service 进入 active
#   3. 等待部署环境稳定
#   4. 启动 start_robot_stack.sh
#   5. 监控 robot_stack 启动进程及其进程组
#   6. systemd 停止服务时，清理整个 robot_stack 进程组
#
# 注意：
#   - 不等待远端 /detect 服务
#   - 不主动启动 qi_deploy.service
#   - robot_stack 意外退出后不会自动重启
#

set -Eeuo pipefail

# ---------- ROS / DDS 环境 ----------
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# 默认使用 ROS domain 22；可由 service Environment 覆盖
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-22}"

# ---------- 路径（相对本脚本，避免项目目录改名后写死路径失效）----------
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# .../qiling_carry_box/src/qiling_s4/scripts/run -> workspace root
readonly PROJECT_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
readonly START_SCRIPT="${SCRIPT_DIR}/start_robot_stack.sh"

# 机载常用 DDS 配置根（与 qi_deploy 交付包一致）
readonly QI_DEPLOY_DDS_DIR="${QI_DEPLOY_DDS_DIR:-/home/coral/qi_deploy_igh_release/config/dds}"
readonly QI_SDK_DDS_DIR="${QI_SDK_DDS_DIR:-/home/coral/ql_sdk_igh_release/config/dds}"

# 该目录由 service 中的 RuntimeDirectory=robot_stack_monitor 创建。
readonly RUNTIME_DIR="/run/robot_stack_monitor"

# 状态文件格式：
# 第一列：启动脚本 PID
# 第二列：启动脚本 PGID
readonly STATE_FILE="${RUNTIME_DIR}/robot_stack.state"

# ---------- 参数 ----------
readonly DEPLOY_SETTLE_SECONDS="${DEPLOY_SETTLE_SECONDS:-30}"
readonly MONITOR_INTERVAL_SECONDS="${MONITOR_INTERVAL_SECONDS:-10}"
readonly STOP_TIMEOUT_SECONDS="${STOP_TIMEOUT_SECONDS:-10}"

# ---------- 全局状态 ----------
CLEANUP_RUNNING=0
STOP_REQUESTED=0
ROBOT_STACK_PID=""
ROBOT_STACK_PGID=""

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

# 去掉 file:// 前缀，得到本地路径（用于可读性检查）
cyclonedds_uri_to_path() {
    local uri="$1"
    if [[ "$uri" == file://* ]]; then
        printf '%s\n' "${uri#file://}"
    else
        printf '%s\n' "$uri"
    fi
}

# 从 SDK yaml 读 network_interface；失败则默认 enp8s0
resolve_dds_network_interface() {
    local yaml="${QI_SDK_DDS_DIR}/dds_config.yaml"
    local iface=""

    if [[ -r "$yaml" ]]; then
        iface="$(
            python3 - <<PY 2>/dev/null || true
import re
text = open(${yaml@Q}, encoding="utf-8").read()
m = re.search(r'^\s*network_interface:\s*["\']?([^"\'#\n]+)', text, re.M)
print(m.group(1).strip() if m else "")
PY
        )"
    fi

    if [[ -z "$iface" ]]; then
        iface="enp8s0"
    fi
    printf '%s\n' "$iface"
}

# 解析可用的 CycloneDDS XML（对齐 qi_deploy：优先交付包按网口命名的 xml）
resolve_cyclonedds_uri() {
    local configured="${CYCLONEDDS_URI:-}"
    local configured_path=""
    local iface=""
    local candidate=""

    if [[ -n "$configured" ]]; then
        configured_path="$(cyclonedds_uri_to_path "$configured")"
        if [[ -r "$configured_path" ]]; then
            if [[ "$configured" == file://* ]]; then
                printf '%s\n' "$configured"
            else
                printf 'file://%s\n' "$configured_path"
            fi
            return 0
        fi
        log "!!! WARNING: 配置的 CYCLONEDDS_URI 不可用: $configured" >&2
    fi

    iface="$(resolve_dds_network_interface)"
    for candidate in \
        "${QI_DEPLOY_DDS_DIR}/cyclonedds_${iface}.xml" \
        "${QI_DEPLOY_DDS_DIR}/cyclonedds_enp8s0.xml" \
        "${QI_DEPLOY_DDS_DIR}/.cyclonedds_runtime.xml" \
        "${QI_SDK_DDS_DIR}/ql_cyclonedds_uri.xml" \
        "${QI_SDK_DDS_DIR}/qi_cyclonedds_uri.xml" \
        "/home/coral/dds/cyclonedds.xml" \
        "/home/coral/project/qi_sdk_internal/install/config/dds/qi_cyclonedds_uri.xml" \
        "${PROJECT_DIR}/src/qiling_s4/qi_ros2/cyclonedds_ws/src/cyclonedds.xml"
    do
        if [[ -r "$candidate" ]]; then
            printf 'file://%s\n' "$candidate"
            return 0
        fi
    done

    return 1
}

# ---------- 参数检查 ----------
validate_positive_integer() {
    local name="$1"
    local value="$2"

    if [[ ! "$value" =~ ^[0-9]+$ ]]; then
        log "!!! FATAL: ${name} 必须是非负整数，当前值: ${value}"
        return 1
    fi
}

validate_configuration() {
    validate_positive_integer \
        "DEPLOY_SETTLE_SECONDS" \
        "$DEPLOY_SETTLE_SECONDS"

    validate_positive_integer \
        "MONITOR_INTERVAL_SECONDS" \
        "$MONITOR_INTERVAL_SECONDS"

    validate_positive_integer \
        "STOP_TIMEOUT_SECONDS" \
        "$STOP_TIMEOUT_SECONDS"

    if (( MONITOR_INTERVAL_SECONDS == 0 )); then
        log "!!! FATAL: MONITOR_INTERVAL_SECONDS 不能为 0"
        return 1
    fi
}

# ---------- 运行目录 ----------
ensure_runtime_directory() {
    if [[ ! -d "$RUNTIME_DIR" ]]; then
        log "!!! FATAL: 运行目录不存在: $RUNTIME_DIR"
        log "!!! 请检查 service 中是否配置 RuntimeDirectory=robot_stack_monitor"
        return 1
    fi

    if [[ ! -w "$RUNTIME_DIR" ]]; then
        log "!!! FATAL: 当前用户无法写入运行目录: $RUNTIME_DIR"
        return 1
    fi
}

# ---------- 状态文件 ----------
remove_state_file() {
    rm -f "$STATE_FILE"
}

write_state_file() {
    local pid="$1"
    local pgid="$2"
    local temp_file="${STATE_FILE}.tmp.$$"

    printf '%s %s\n' "$pid" "$pgid" > "$temp_file"
    mv -f "$temp_file" "$STATE_FILE"
}

read_state_file() {
    local pid=""
    local pgid=""

    [[ -f "$STATE_FILE" ]] || return 1

    read -r pid pgid < "$STATE_FILE" || return 1

    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    [[ "$pgid" =~ ^[0-9]+$ ]] || return 1

    printf '%s %s\n' "$pid" "$pgid"
}

load_state_file() {
    local state=""

    state="$(read_state_file)" || return 1
    read -r ROBOT_STACK_PID ROBOT_STACK_PGID <<< "$state"

    return 0
}

# ---------- 进程检查 ----------
is_pid_alive() {
    local pid="$1"
    local process_state=""

    [[ "$pid" =~ ^[0-9]+$ ]] || return 1

    kill -0 "$pid" 2>/dev/null || return 1

    # kill -0 对僵尸进程也可能返回成功，因此额外检查进程状态。
    process_state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]')" || true

    [[ -n "$process_state" ]] || return 1
    [[ "$process_state" != Z* ]]
}

is_process_group_alive() {
    local pgid="$1"

    [[ "$pgid" =~ ^[0-9]+$ ]] || return 1

    # 向负 PGID 发送信号表示操作整个进程组。
    kill -0 -- "-${pgid}" 2>/dev/null
}

is_robot_stack_alive() {
    local pid=""
    local pgid=""
    local state=""

    state="$(read_state_file 2>/dev/null)" || return 1
    read -r pid pgid <<< "$state"

    if ! is_pid_alive "$pid"; then
        return 1
    fi

    if ! is_process_group_alive "$pgid"; then
        return 1
    fi

    return 0
}

remove_stale_state_file() {
    if [[ ! -f "$STATE_FILE" ]]; then
        return 0
    fi

    if is_robot_stack_alive; then
        return 0
    fi

    log ">>> 检测到失效的 robot_stack 状态文件，正在删除"
    remove_state_file
}

# ---------- 停止进程组 ----------
stop_process_group() {
    local pgid="$1"
    local elapsed=0

    [[ "$pgid" =~ ^[0-9]+$ ]] || return 0

    if ! is_process_group_alive "$pgid"; then
        return 0
    fi

    log ">>> 向 robot_stack 进程组发送 SIGINT (PGID: $pgid)"
    kill -INT -- "-${pgid}" 2>/dev/null || true

    while is_process_group_alive "$pgid"; do
        if (( elapsed >= STOP_TIMEOUT_SECONDS )); then
            log ">>> robot_stack 未在 ${STOP_TIMEOUT_SECONDS}s 内退出，发送 SIGKILL"
            kill -KILL -- "-${pgid}" 2>/dev/null || true
            break
        fi

        sleep 1
        ((elapsed += 1))
    done

    # 给内核一点时间回收进程状态。
    sleep 1
}

# ---------- 信号与清理 ----------
handle_stop_signal() {
    STOP_REQUESTED=1
    exit 0
}

cleanup() {
    local exit_code=$?

    if (( CLEANUP_RUNNING == 1 )); then
        return
    fi

    CLEANUP_RUNNING=1

    trap - SIGTERM SIGINT EXIT

    if (( STOP_REQUESTED == 1 )); then
        log ">>> 收到 systemd 停止信号，开始清理"
    elif (( exit_code != 0 )); then
        log ">>> 监控脚本异常退出，开始清理"
    else
        log ">>> 监控脚本退出，开始清理"
    fi

    if load_state_file 2>/dev/null; then
        stop_process_group "$ROBOT_STACK_PGID"

        # 启动脚本是当前监控脚本的子进程，尝试回收退出状态。
        wait "$ROBOT_STACK_PID" 2>/dev/null || true
    fi

    remove_state_file

    log ">>> robot_stack 监控进程已停止"

    exit "$exit_code"
}

trap handle_stop_signal SIGTERM SIGINT
trap cleanup EXIT

# ---------- qi_deploy.service ----------
is_deploy_ready() {
    systemctl is-active --quiet qi_deploy.service
}

wait_for_deploy() {
    local wait_count=0

    log ">>> 等待 qi_deploy.service 进入 active 状态"

    until is_deploy_ready; do
        ((wait_count += 1))

        if (( wait_count == 1 || wait_count % 20 == 0 )); then
            log ">>> qi_deploy.service 尚未启动，已等待约 $((wait_count * 3))s"
        fi

        sleep 3
    done

    log ">>> qi_deploy.service 已启动"

    if (( DEPLOY_SETTLE_SECONDS > 0 )); then
        log ">>> 等待 ${DEPLOY_SETTLE_SECONDS}s，让部署环境完全就绪"
        sleep "$DEPLOY_SETTLE_SECONDS"
    fi

    # 等待期间 qi_deploy 可能退出，因此再次确认。
    if ! is_deploy_ready; then
        log "!!! FATAL: qi_deploy.service 在稳定等待期间退出"
        return 1
    fi

    log ">>> 部署环境等待完成"
}

# ---------- ROS 环境 ----------
load_ros_environment() {
    local ros_setup="/opt/ros/humble/setup.bash"
    local workspace_setup="${PROJECT_DIR}/install/setup.bash"
    local dds_path=""

    if [[ ! -r "$ros_setup" ]]; then
        log "!!! FATAL: 找不到 ROS 环境文件: $ros_setup"
        return 1
    fi

    if [[ ! -r "$workspace_setup" ]]; then
        log "!!! FATAL: 找不到工作空间环境文件: $workspace_setup"
        return 1
    fi

    # ROS setup 脚本可能引用尚未定义的变量，临时关闭 nounset。
    set +u

    # shellcheck disable=SC1091
    source "$ros_setup"

    # shellcheck disable=SC1091
    source "$workspace_setup"

    set -u

    if ! command -v ros2 >/dev/null 2>&1; then
        log "!!! FATAL: ros2 命令不可用"
        return 1
    fi

    if ! CYCLONEDDS_URI="$(resolve_cyclonedds_uri)"; then
        log "!!! FATAL: 找不到可用的 CycloneDDS 配置文件"
        log "!!! 已检查: ${QI_DEPLOY_DDS_DIR}/cyclonedds_*.xml"
        log "!!! 以及: ${QI_SDK_DDS_DIR}/ql_cyclonedds_uri.xml"
        log "!!! 也可在 service 中设置 CYCLONEDDS_URI=file:///绝对路径/xxx.xml"
        return 1
    fi
    export CYCLONEDDS_URI
    dds_path="$(cyclonedds_uri_to_path "$CYCLONEDDS_URI")"

    log ">>> ROS 环境加载完成"
    log ">>> PROJECT_DIR=$PROJECT_DIR"
    log ">>> RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
    log ">>> ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    log ">>> CYCLONEDDS_URI=$CYCLONEDDS_URI"
    log ">>> CycloneDDS file=$dds_path"
}

# ---------- 启动 robot_stack ----------
start_robot_stack() {
    local launch_pid=""
    local launch_pgid=""

    if [[ ! -x "$START_SCRIPT" ]]; then
        log "!!! FATAL: 启动脚本不存在或不可执行: $START_SCRIPT"
        return 1
    fi

    if is_robot_stack_alive; then
        load_state_file
        log ">>> robot_stack 已经在运行，跳过重复启动"
        log ">>> PID:  $ROBOT_STACK_PID"
        log ">>> PGID: $ROBOT_STACK_PGID"
        return 0
    fi

    remove_state_file

    log ">>> 启动 robot_stack"

    cd "$PROJECT_DIR"

    # 为 start_robot_stack.sh 创建独立 session 和进程组。
    # start_robot_stack.sh 自身负责启动 ros2 launch 并等待其退出。
    setsid "$START_SCRIPT" &

    launch_pid=$!

    # 等待启动脚本建立进程组，并检测是否立即失败。
    sleep 1

    if ! is_pid_alive "$launch_pid"; then
        wait "$launch_pid" 2>/dev/null || true
        log "!!! FATAL: robot_stack 启动后立即退出"
        return 1
    fi

    launch_pgid="$(
        ps -o pgid= -p "$launch_pid" 2>/dev/null |
            tr -d '[:space:]'
    )"

    if [[ ! "$launch_pgid" =~ ^[0-9]+$ ]]; then
        log "!!! FATAL: 无法获取 robot_stack 启动脚本的 PGID"
        kill -TERM "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
        return 1
    fi

    if ! is_process_group_alive "$launch_pgid"; then
        log "!!! FATAL: robot_stack 进程组创建失败"
        kill -TERM "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
        return 1
    fi

    write_state_file "$launch_pid" "$launch_pgid"

    ROBOT_STACK_PID="$launch_pid"
    ROBOT_STACK_PGID="$launch_pgid"

    log ">>> robot_stack 已启动"
    log ">>> PID:  $ROBOT_STACK_PID"
    log ">>> PGID: $ROBOT_STACK_PGID"
    log ">>> 状态文件: $STATE_FILE"
}

# ---------- 监控 ----------
monitor_robot_stack() {
    if ! load_state_file; then
        log "!!! FATAL: 无法读取 robot_stack 状态文件"
        return 1
    fi

    log ">>> 开始监控 robot_stack"
    log ">>> PID:  $ROBOT_STACK_PID"
    log ">>> PGID: $ROBOT_STACK_PGID"

    while true; do
        sleep "$MONITOR_INTERVAL_SECONDS"

        if ! is_pid_alive "$ROBOT_STACK_PID"; then
            log "!!! ========================================"
            log "!!! WARNING: robot_stack 启动脚本已退出"
            log "!!! 当前配置不会自动重启"
            log "!!! 手动重启命令："
            log "!!! sudo systemctl restart robot_stack_monitor"
            log "!!! ========================================"

            remove_state_file

            # robot_stack 已经退出，不需要 EXIT trap 再次清理该进程组。
            ROBOT_STACK_PID=""
            ROBOT_STACK_PGID=""

            return 1
        fi

        if ! is_process_group_alive "$ROBOT_STACK_PGID"; then
            log "!!! ========================================"
            log "!!! WARNING: robot_stack 进程组已退出"
            log "!!! 当前配置不会自动重启"
            log "!!! 手动重启命令："
            log "!!! sudo systemctl restart robot_stack_monitor"
            log "!!! ========================================"

            remove_state_file
            ROBOT_STACK_PID=""
            ROBOT_STACK_PGID=""

            return 1
        fi

        # qi_deploy 掉线时只记录警告，不立即停止 robot_stack。
        if ! is_deploy_ready; then
            log "!!! WARNING: qi_deploy.service 当前不是 active 状态"
        fi
    done
}

main() {
    log "╔══════════════════════════════════════════╗"
    log "║  robot_stack 监控 v2.1                  ║"
    log "║  qi_deploy 就绪后自动启动 robot_stack   ║"
    log "╚══════════════════════════════════════════╝"

    validate_configuration
    ensure_runtime_directory
    remove_stale_state_file
    load_ros_environment
    wait_for_deploy
    start_robot_stack
    monitor_robot_stack
}

main "$@"
