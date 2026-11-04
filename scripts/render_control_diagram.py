"""Render the PiPER single-joint servo control block diagram.

The figure is a classic three-loop servo diagram (position -> velocity ->
current/FOC) with the PiPER-specific interfaces annotated. It is generated
offline and does not touch the robot.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch

OUTPUT = Path(__file__).resolve().parents[1] / "docs" / "piper_servo_control_logic.png"

plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


def box(ax, x, y, w, h, text, fontsize=11, fc="#ffffff", lw=1.6):
    patch = FancyBboxPatch(
        (x, y - h / 2), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.35",
        linewidth=lw, edgecolor="black", facecolor=fc, zorder=3,
    )
    ax.add_patch(patch)
    ax.text(x + w / 2, y, text, ha="center", va="center",
            fontsize=fontsize, zorder=4)
    return (x, x + w)


def arrow(ax, start, end, lw=1.5, style="-|>", color="black"):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle=style, mutation_scale=14,
        linewidth=lw, color=color, shrinkA=0, shrinkB=0, zorder=2,
    ))


def wire(ax, points, lw=1.4, color="black", end_arrow=True):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ax.plot(xs[:-1] + [xs[-1]], ys[:-1] + [ys[-1]], color=color, lw=lw, zorder=2,
            solid_capstyle="round")
    if end_arrow:
        arrow(ax, points[-2], points[-1], lw=lw, color=color)


def summing(ax, x, y, r=1.55):
    ax.add_patch(Circle((x, y), r, facecolor="white", edgecolor="black",
                        linewidth=1.5, zorder=3))
    ax.text(x - r - 0.55, y + 0.55, "+", fontsize=12, ha="center", va="center", zorder=4)
    ax.text(x - 0.95, y - 2.35, "−", fontsize=14, ha="center", va="center", zorder=4)


def main() -> None:
    fig, ax = plt.subplots(figsize=(15.5, 8.6))
    ax.set_xlim(0, 128)
    ax.set_ylim(0, 72)
    ax.axis("off")

    ax.text(64, 69.5,
            "AgileX PiPER 单关节伺服控制逻辑（三环 + 力矩前馈）",
            ha="center", va="center", fontsize=16, fontweight="bold")

    # ---- main forward path -------------------------------------------------
    box(ax, 2, 45, 12, 10, "上位机 / SDK\n指令", fontsize=10.5)
    summing(ax, 18, 45)
    box(ax, 21, 45, 12, 10, "位置调节\nP  (pp)")
    summing(ax, 35, 45)
    box(ax, 38, 45, 13, 10, "速度调节\nPI (kp, ki)")
    summing(ax, 54, 45)
    box(ax, 57, 45, 14, 10, "电流调节\n+ FOC 功率放大")
    box(ax, 74, 45, 15, 10, "伺服电机\n+ 减速器")

    arrow(ax, (14, 45), (16.45, 45))
    arrow(ax, (19.55, 45), (21, 45))
    arrow(ax, (33, 45), (33.45, 45))
    arrow(ax, (36.55, 45), (38, 45))
    arrow(ax, (51, 45), (52.45, 45))
    arrow(ax, (55.55, 45), (57, 45))
    arrow(ax, (71, 45), (74, 45))
    arrow(ax, (89, 45), (98, 45))
    ax.text(99, 47.2, "关节输出", fontsize=11, va="bottom")
    ax.text(99, 42.8, "q, τ", fontsize=11, va="top")

    ax.text(14.8, 46.6, "q_des", fontsize=9, ha="left")
    ax.text(33.2, 46.6, "v_ref", fontsize=9, ha="left")
    ax.text(56.2, 46.6, "i_ref", fontsize=9, ha="left")
    ax.text(72.3, 46.6, "τ_motor", fontsize=9, ha="left")

    # external load disturbance
    arrow(ax, (81.5, 55.5), (81.5, 50.2))
    ax.text(82.2, 56.0, "负载 / 扰动", fontsize=9, ha="left", va="center")

    # ---- feedforward: t_ff / gravity --------------------------------------
    box(ax, 47, 58, 14, 7.5, "力矩前馈\nt_ff / τ_g", fontsize=10, fc="#fff2cc")
    arrow(ax, (54, 54.25), (54, 46.55))
    ax.text(46.3, 57.0, "MIT: kp=kd=0 时\n为纯力矩前馈", fontsize=8.5,
            ha="right", va="center")

    # ---- feedback: encoder -------------------------------------------------
    box(ax, 74, 18, 15, 6.5, "编码器 (q)", fontsize=10, fc="#e8f0fe")
    arrow(ax, (81.5, 40), (81.5, 21.25))

    # position feedback: encoder -> sum1
    wire(ax, [(74, 18), (18, 18), (18, 43.45)])
    ax.text(44, 15.9, "位置反馈 q", fontsize=9, ha="center")

    # derivative (tachometer) -> sum2
    box(ax, 30, 27, 10, 5.5, "微分 / 测速", fontsize=9.5, fc="#e8f0fe")
    wire(ax, [(35, 18), (35, 24.25)])
    arrow(ax, (35, 29.25), (35, 43.45))
    ax.text(36.1, 25.2, "v", fontsize=9, ha="left")

    # current feedback -> sum3
    wire(ax, [(72.5, 45), (72.5, 34), (54, 34), (54, 43.45)])
    ax.text(60, 32.1, "电流反馈 i", fontsize=9, ha="center")

    # ---- CPV reference shaping note ---------------------------------------
    box(ax, 2, 30, 22, 7.5,
        "CPV 模式：位置/速度指令\nac/dc/vv 生成参考轨迹", fontsize=9, fc="#eaf4ea")
    arrow(ax, (24, 33), (29.5, 40), lw=1.2, style="-|>")
    ax.text(25.5, 37.2, "参考轨迹\n（前馈）", fontsize=8, ha="left")

    # ---- MIT formula inset -------------------------------------------------
    ax.add_patch(FancyBboxPatch(
        (2, 2), 58, 9.5, boxstyle="round,pad=0.05,rounding_size=0.4",
        linewidth=1.2, edgecolor="black", facecolor="#f7f7f7", zorder=1,
    ))
    ax.text(3.5, 8.2,
            "MIT 力矩参考：  T_ref = kp (p_des − p) + kd (v_des − v) + t_ff",
            fontsize=10, va="center")
    ax.text(3.5, 4.8,
            "电流换算：  i_ref = T_ref / (k·b·c)，  关节力矩 ≈ 电流 × k·b·c",
            fontsize=10, va="center")

    # ---- interface legend --------------------------------------------------
    ax.add_patch(FancyBboxPatch(
        (96, 2), 29, 26, boxstyle="round,pad=0.05,rounding_size=0.4",
        linewidth=1.2, edgecolor="black", facecolor="#fafafa", zorder=1,
    ))
    ax.text(98, 25.5, "PiPER 接口", fontsize=11, fontweight="bold", va="top")
    lines = [
        "模式：0x151",
        "位置：0x155–0x157 (0.001°)",
        "CPV：0x181–0x186 (po/sp)",
        "MIT：0x15A–0x15F (t_ff)",
        "位置反馈：0x2A5–0x2A7",
        "速度/电流：0x251–0x256",
        "电流→力矩：×k·b·c",
    ]
    for index, line in enumerate(lines):
        ax.text(98, 21.8 - index * 2.75, "• " + line, fontsize=8.6, va="center")

    fig.tight_layout()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT, dpi=200, bbox_inches="tight")
    fig.savefig(OUTPUT.with_suffix(".svg"), bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUTPUT} and {OUTPUT.with_suffix('.svg')}")


if __name__ == "__main__":
    main()
