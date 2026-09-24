#!/usr/bin/env python3
"""IK 扫描结果 3D 可视化。

读取 ik_sweep_tester 输出的 CSV，生成交互式 3D 散点图和分截面热力图。

用法：
  python3 visualize_ik_sweep.py /tmp/ik_sweep_results.csv
  python3 visualize_ik_sweep.py /tmp/ik_sweep_results.csv --output ik_sweep.png
"""

import csv
import sys
from argparse import ArgumentParser
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
import matplotlib.font_manager as fm

# ---- 中文字体 ----
_CJK_CANDIDATES = [
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
    "SimHei",
    "AR PL UMing CN",
    "AR PL UKai CN",
]
_FOUND = False
for _fname in _CJK_CANDIDATES:
    _match = [f for f in fm.fontManager.ttflist if _fname in f.name]
    if _match:
        matplotlib.rcParams["font.family"] = _match[0].name
        matplotlib.rcParams["axes.unicode_minus"] = False
        _FOUND = True
        break
if not _FOUND:
    print("警告：未找到中文字体，图形中的中文可能显示为方块。")

import numpy as np

# --- 数据读取 ---


def load_csv(csv_path: str) -> List[dict]:
    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "grasp_ok": row["grasp_ok"].strip().lower() in ("true", "1", "yes"),
                "grasp_msg": row.get("grasp_msg", ""),
                "lift_ok": row.get("lift_ok", "").strip().lower() in ("true", "1", "yes"),
                "lift_msg": row.get("lift_msg", ""),
                "all_ok": row["all_ok"].strip().lower() in ("true", "1", "yes"),
            })
    return rows


def failure_categories(rows: List[dict]) -> Dict[str, int]:
    cats: Dict[str, int] = {}
    for r in rows:
        if r["all_ok"]:
            continue
        if not r["grasp_ok"]:
            msg = r["grasp_msg"]
        else:
            msg = r["lift_msg"]
        # 简化分类
        if "timeout" in msg.lower():
            key = "timeout"
        elif "rejected" in msg.lower():
            key = "rejected"
        elif any(kw in msg for kw in ("跳变", "jump", "joint_step", "连续性")):
            key = "IK连续性(jump)"
        elif any(kw in msg for kw in ("失败", "fail", "FAILED", "IK", "无解", "无逆解")):
            key = "IK无解"
        else:
            key = "其他"
        cats[key] = cats.get(key, 0) + 1
    return cats


# --- Matplotlib 3D 可视化 ---


def plot_3d(rows: List[dict], output_png: str = "") -> None:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - 注册 3D projection

    xs_ok, ys_ok, zs_ok = [], [], []
    xs_fail, ys_fail, zs_fail = [], [], []
    xs_only_grasp, ys_only_grasp, zs_only_grasp = [], [], []

    for r in rows:
        if r["all_ok"]:
            xs_ok.append(r["x"]); ys_ok.append(r["y"]); zs_ok.append(r["z"])
        else:
            xs_fail.append(r["x"]); ys_fail.append(r["y"]); zs_fail.append(r["z"])
            if r["grasp_ok"] and not r["lift_ok"]:
                xs_only_grasp.append(r["x"]); ys_only_grasp.append(r["y"]); zs_only_grasp.append(r["z"])

    total = len(rows)
    all_ok = len(xs_ok)
    grasp_only = len(xs_only_grasp)

    fig = plt.figure(figsize=(16, 7))

    # ---- 左图：3D 散点图 ----
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    if xs_ok:
        ax.scatter(xs_ok, ys_ok, zs_ok, c="limegreen", s=25, alpha=0.8, label=f"全部成功 ({all_ok})")
    if xs_only_grasp:
        ax.scatter(xs_only_grasp, ys_only_grasp, zs_only_grasp,
                   c="orange", s=30, marker="^", alpha=0.9, label=f"仅抓取成功 ({grasp_only})")
    if xs_fail:
        # 将纯失败（抓取都失败）区分开
        xs_pure_fail = [x for i, x in enumerate(xs_fail)
                        if not (rows[i]["grasp_ok"] and not rows[i]["lift_ok"])]
        ys_pure_fail = [y for i, y in enumerate(ys_fail)
                        if not (rows[i]["grasp_ok"] and not rows[i]["lift_ok"])]
        zs_pure_fail = [z for i, z in enumerate(zs_fail)
                        if not (rows[i]["grasp_ok"] and not rows[i]["lift_ok"])]
        count_pure = len(xs_pure_fail)
        if xs_pure_fail:
            ax.scatter(xs_pure_fail, ys_pure_fail, zs_pure_fail,
                       c="red", s=30, marker="x", alpha=0.7, label=f"抓取失败 ({count_pure})")

    ax.set_xlabel("X (base_link)")
    ax.set_ylabel("Y (base_link)")
    ax.set_zlabel("Z (base_link)")
    ax.set_title(f"IK 三段成功率：{all_ok}/{total} = {100 * all_ok / total:.1f}%")
    ax.legend(loc="upper left", fontsize=8)

    # ---- 右图：分 Z 截面柱状图 ----
    z_slices: Dict[float, Tuple[int, int, int]] = {}
    for r in rows:
        z = r["z"]
        if z not in z_slices:
            z_slices[z] = (0, 0, 0)
        t, ok, grasp = z_slices[z]
        z_slices[z] = (t + 1, ok + (1 if r["all_ok"] else 0), grasp + (1 if r["grasp_ok"] else 0))

    z_vals = sorted(z_slices.keys())
    totals = [z_slices[z][0] for z in z_vals]
    all_oks = [z_slices[z][1] for z in z_vals]
    grasp_oks = [z_slices[z][2] for z in z_vals]
    fail_grasps = [t - g for t, _, g in [(z_slices[z][0], 0, z_slices[z][2]) for z in z_vals]]
    lift_only_fails = [g - a for _, a, g in [(z_slices[z][1], z_slices[z][1], z_slices[z][2]) for z in z_vals]]

    ax2 = fig.add_subplot(1, 2, 2)
    z_labels = [f"{z:+.2f}" for z in z_vals]
    x_idx = np.arange(len(z_vals))
    width = 0.35

    bars_all = ax2.bar(x_idx - width / 2, all_oks, width, color="limegreen", label="三段全成功")
    bars_grasp = ax2.bar(x_idx + width / 2, grasp_oks, width, color="royalblue", alpha=0.7, label="抓取段成功")

    # 在柱子上标注百分比
    for i, (t, ok) in enumerate(zip(totals, all_oks)):
        ax2.text(x_idx[i] - width / 2, ok + 1, f"{100 * ok / t:.0f}%", ha="center", fontsize=7)
    for i, (t, gok) in enumerate(zip(totals, grasp_oks)):
        ax2.text(x_idx[i] + width / 2, gok + 1, f"{100 * gok / t:.0f}%", ha="center", fontsize=7)

    ax2.set_xlabel("Z 截面 (base_link)")
    ax2.set_ylabel("成功点数")
    ax2.set_title(f"各 Z 截面成功率（每截面 {totals[0] if totals else 0} 点）")
    ax2.set_xticks(x_idx)
    ax2.set_xticklabels(z_labels, rotation=45)
    ax2.legend(fontsize=8)
    ax2.set_ylim(0, max(totals) * 1.25)

    # 失败分类
    cats = failure_categories(rows)
    if cats:
        cat_text = "\n".join(f"  {k}: {v}" for k, v in sorted(cats.items(), key=lambda x: -x[1]))
        fig.text(0.02, 0.02, f"失败原因分布：\n{cat_text}", fontsize=8, family="monospace",
                 bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.8))

    fig.suptitle(f"IK 扫描结果：{total} 个点，三段全部成功 {all_ok} 个 ({100 * all_ok / total:.1f}%)",
                 fontweight="bold")
    plt.tight_layout(rect=[0, 0.12, 1, 0.95])

    if output_png:
        plt.savefig(output_png, dpi=150, bbox_inches="tight")
        print(f"图片已保存到：{output_png}")
    else:
        plt.show()


def main() -> None:
    parser = ArgumentParser(description="IK 扫描结果 3D 可视化")
    parser.add_argument("csv_path", help="CSV 文件路径（ik_sweep_tester 输出）")
    parser.add_argument("--output", "-o", default="", help="输出 PNG 图片路径（默认显示窗口）")
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        print(f"错误：文件不存在 {csv_path}")
        sys.exit(1)

    rows = load_csv(str(csv_path))
    if not rows:
        print("CSV 为空。")
        sys.exit(1)

    total = len(rows)
    grasp_ok = sum(1 for r in rows if r["grasp_ok"])
    lift_ok = sum(1 for r in rows if r["lift_ok"])
    all_ok = sum(1 for r in rows if r["all_ok"])

    print(f"加载了 {total} 个测试点。")
    print(f"  抓取段成功:  {grasp_ok}/{total} = {100 * grasp_ok / total:.1f}%")
    print(f"  lift/place:  {lift_ok}/{grasp_ok} = {100 * lift_ok / grasp_ok:.1f}% (基于抓取成功)")
    print(f"  三段全成功:  {all_ok}/{total} = {100 * all_ok / total:.1f}%")

    cats = failure_categories(rows)
    if cats:
        print("\n失败原因分布：")
        for k, v in sorted(cats.items(), key=lambda x: -x[1]):
            print(f"  {k}: {v}")

    try:
        plot_3d(rows, args.output)
    except ImportError:
        print("\n需要 matplotlib 和 numpy：pip install matplotlib numpy")
        sys.exit(1)


if __name__ == "__main__":
    main()
