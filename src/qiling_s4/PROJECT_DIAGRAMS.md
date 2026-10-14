# qiling_s4 项目流程图

本文档用 Mermaid 描述 qiling_s4 当前上肢控制项目的核心流程。图中只保留关键模块、关键数据和主要阶段，便于快速理解整体系统。

## 软件架构图

```mermaid
flowchart TB
    User["用户 / 任务侧<br/>抓取、放置、手动目标"] --> Interface["ROS2 接口层<br/>grasp_command / place_command<br/>MoveItArmGoal"]

    Interface --> Task["任务状态机<br/>vision_grasp_sequence"]
    Task --> Vision["视觉检测服务<br/>detect"]
    Task --> Planner["MoveIt 规划控制<br/>moveit_goal_commander"]
    Task --> Hand["灵巧手控制<br/>/handscmd"]

    Planner --> MoveIt["MoveIt Core<br/>IK / 规划 / 轨迹缓存"]
    Planner --> Bridge["轨迹执行桥<br/>moveit_mit_arm_bridge"]

    State["机器人状态输入<br/>lowstate 或外部 /joint_states"] --> JointState["/joint_states"]
    JointState --> MoveIt
    JointState --> Planner
    JointState --> Bridge

    Bridge --> LowCmd["lowcmd / mode_ctrl=5<br/>DDS rt/lowcmd"]
    LowCmd --> Robot["真机 SDK / 外部仿真"]
    Hand --> Robot
```

## 数据流图

```mermaid
flowchart LR
    subgraph Input["输入"]
        A["抓取/放置命令"]
        B["视觉 box pose<br/>或手动 box pose"]
        C["关节状态 /joint_states"]
    end

    subgraph Process["处理"]
        D["任务状态机<br/>解析命令与流程控制"]
        E["目标生成<br/>左右末端位姿 / 关节目标"]
        F["MoveIt 规划<br/>IK、轨迹、缓存"]
        G["轨迹桥接<br/>轨迹插值、PD、重力补偿"]
    end

    subgraph Output["输出"]
        H["lowcmd / mode_ctrl=5<br/>双臂控制"]
        I["/handscmd<br/>灵巧手开合"]
        J["状态/诊断<br/>/vision_grasp/status<br/>/both_arms/result"]
    end

    A --> D
    B --> D
    C --> F
    C --> G
    D --> E --> F --> G --> H
    D --> I
    D --> J
    F --> J
    G --> J
```

## 抓取算法流程图

```mermaid
flowchart TD
    A([收到 grasp 命令]) --> B{命令类型}

    B -->|grasp| C["调用视觉检测<br/>获取箱体位置"]
    B -->|grasp x y z| D["使用手动箱体坐标"]
    B -->|grasp_joints| E["使用调试好的14关节角"]

    C --> F["生成左右臂抓取目标"]
    D --> F
    E --> G["根据关节角预规划上抬/下降"]

    F --> H["预规划抓取轨迹"]
    H --> I["预规划上抬/下降轨迹"]
    I --> J["执行预抓取过渡"]
    G --> J

    J --> K["执行抓取动作"]
    K --> L["闭合灵巧手"]
    L --> M["执行上抬轨迹"]
    M --> N{是否等待 place?}

    N -->|是| O["保持等待放置命令"]
    N -->|否| P["自动执行下降/放置"]

    O --> Q([收到 place 命令])
    Q --> R["执行缓存下降轨迹"]
    P --> R
    R --> S["张开灵巧手"]
    S --> T["回 home"]
    T --> U([流程结束])
```

## 运行时序图

```mermaid
sequenceDiagram
    autonumber
    actor User as 用户
    participant Seq as 抓取状态机
    participant Vision as 视觉服务
    participant Planner as MoveIt规划节点
    participant Bridge as 轨迹桥
    participant Robot as 真机/仿真
    participant Hand as 灵巧手

    User->>Seq: 发送 grasp 命令
    alt 视觉抓取
        Seq->>Vision: 请求检测箱体
        Vision-->>Seq: 返回 box pose
    else 手动/关节角抓取
        Seq->>Seq: 解析命令参数
    end

    Seq->>Planner: 请求抓取/上抬预规划
    Planner-->>Seq: 返回规划结果
    Seq->>Planner: 执行预抓取与抓取轨迹
    Planner->>Bridge: 下发 FollowJointTrajectory
    Bridge->>Robot: 持续发布 lowcmd (mode_ctrl=5)
    Robot-->>Bridge: 执行结果
    Bridge-->>Planner: action result
    Planner-->>Seq: /both_arms/result

    Seq->>Hand: 闭合灵巧手
    Seq->>Planner: 执行上抬轨迹
    Planner->>Bridge: 下发缓存轨迹
    Bridge->>Robot: lowcmd (mode_ctrl=5)
    Planner-->>Seq: 上抬完成

    User->>Seq: 发送 place 命令
    Seq->>Planner: 执行缓存下降轨迹
    Planner->>Bridge: 下发轨迹
    Bridge->>Robot: lowcmd (mode_ctrl=5)
    Planner-->>Seq: 下降完成
    Seq->>Hand: 张开灵巧手
    Seq->>Planner: 回 home
    Planner-->>Seq: 流程完成
```

## 真机与外部仿真链路

```mermaid
flowchart TB
    subgraph Real["真机"]
        R1["SDK lowstate"] --> R2["lowstate_joint_states_bridge"]
        R2 --> R3["/joint_states"]
        R3 --> R4["MoveIt + 轨迹桥"]
        R4 --> R5["lowcmd / mode_ctrl=5<br/>(DDS rt/lowcmd)"]
        R5 --> R6["SDK 执行上肢控制"]
    end

    subgraph Sim["外部仿真"]
        S1["外部仿真发布 /joint_states"] --> S2["MoveIt + 轨迹桥"]
        S2 --> S3["lowcmd"]
        S3 --> S4["仿真 adapter"]
    end
```

## 说明

- 当前项目主职责是双臂上肢控制；下肢控制不在 qiling_s4 侧处理。
- 真机向 `lowcmd`（DDS `rt/lowcmd`）发全身长度 `LowCmd` 且设置 `mode_ctrl=5`；SDK 用已缓存的策略腿替换包内腿部占位数据。
- `/handsstate` 当前只缓存，不作为闭环控制依据；灵巧手动作由 `/handscmd` 开环下发。
- 真机和仿真都依赖唯一有效的 `/joint_states`，不要同时启动多个状态源。
