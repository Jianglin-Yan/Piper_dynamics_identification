"""Tables, figures and interpretation of an offline trajectory audit."""

from __future__ import annotations

import csv
import json

import numpy as np

LABELS = {
    "old_fast_0": "原轨迹快",
    "old_slow_0": "原轨迹慢",
    "balanced_v1": "优化v1",
    "balanced_v2": "优化v2",
}
PRIMARY = list(LABELS)


def write_csv_files(result, output):
    """将既有审计结果写成角度/速度、信息量及非线性敏感度三张CSV表。"""
    rows = []
    information = []
    nonlinear = []
    for row in result["dynamic"]:
        for j, speed in enumerate(row["speed_position_sg21"]["joints"]):
            rows.append(
                {
                    "dataset": row["name"],
                    "joint": j + 1,
                    "q_min_deg": row["coverage"]["min_deg"][j],
                    "q_max_deg": row["coverage"]["max_deg"][j],
                    "joint_range_percent": 100 * row["coverage"]["range_fraction"][j],
                    "peak_speed_rad_s": speed["peak_rad_s"],
                    "low_0p002_to_0p02_percent": 100 * speed["low_moving_fraction"],
                    "above_0p2_percent": 100 * speed["above_0p2_fraction"],
                    "low_stable_seconds": speed["stable_low_residence"]["seconds"],
                    "low_stable_longest_s": speed["stable_low_residence"][
                        "longest_visit_s"
                    ],
                    "low_stable_visits_over_0p25s": speed["stable_low_residence"][
                        "visits_at_least_0p25s"
                    ],
                }
            )
        for scope, info in row["information"].items():
            information.append(
                {
                    "dataset": row["name"],
                    "scope": scope,
                    **{k: v for k, v in info.items() if k != "singular_values"},
                }
            )
        for width, joints in row["stribeck_sensitivity"].items():
            for j, diagnostics in enumerate(joints):
                nonlinear.extend(
                    {"dataset": row["name"], "joint": j + 1, **x} for x in diagnostics
                )
    for name, values in [
        ("joint_metrics.csv", rows),
        ("information.csv", information),
        ("stribeck_sensitivity.csv", nonlinear),
    ]:
        with (output / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(values[0]))
            writer.writeheader()
            writer.writerows(values)


def plot_coverage(result, plt, output):
    """输出逐轴角度范围比例热图；明确区分边际行程与空间体积覆盖。"""
    rows = result["dynamic"]
    fractions = 100 * np.array([r["coverage"]["range_fraction"] for r in rows])
    fig, ax = plt.subplots(figsize=(10, 5.8))
    image = ax.imshow(fractions, cmap="YlGnBu", vmin=0, vmax=50, aspect="auto")
    ax.set_xticks(range(6), [f"J{j}" for j in range(1, 7)])
    ax.set_yticks(range(len(rows)), [LABELS.get(r["name"], r["name"]) for r in rows])
    for i in range(len(rows)):
        for j in range(6):
            ax.text(
                j,
                i,
                f"{fractions[i, j]:.1f}%",
                ha="center",
                va="center",
                color="white" if fractions[i, j] > 30 else "#143246",
            )
    fig.colorbar(image, ax=ax, label="实测峰峰行程 / 固件关节总行程 · %")
    ax.set_title("8条实机激励轨迹：逐轴角度范围覆盖\n不是六维空间或末端工作空间覆盖率")
    fig.tight_layout()
    fig.savefig(output / "joint_coverage.png")
    plt.close(fig)


def plot_speed(result, plt, output):
    """输出四条主轨迹的逐轴速度分箱图，使用结果中既定的稳态与有效掩码。"""
    primary = [
        next(r for r in result["dynamic"] if r["name"] == name) for name in PRIMARY
    ]
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharex=True, sharey=True)
    for j, ax in enumerate(axes.flat):
        for row in primary:
            fractions = row["speed_position_sg21"]["joints"][j]["bin_fractions"]
            ax.plot(
                np.arange(10),
                100 * np.array(fractions[:10]),
                "-o",
                ms=3,
                label=LABELS[row["name"]],
            )
        ax.set_title(f"J{j + 1}")
        ax.set_xticks(
            np.arange(10),
            [
                "<.002",
                ".002–.005",
                ".005–.01",
                ".01–.02",
                ".02–.05",
                ".05–.1",
                ".1–.2",
                ".2–.5",
                ".5–1",
                "1–2",
            ],
            rotation=65,
        )
        ax.set_ylabel("有效稳态时间占比 / %")
        ax.grid(alpha=0.2)
    axes[0, 0].legend(fontsize=8, frameon=False)
    fig.suptitle(
        "速度分布：由实测位置微分，剔除启动/结束包络与数据缺口\n分箱为诊断口径；0.2 rad/s不是额定高速定义"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(output / "velocity_coverage.png")
    plt.close(fig)


def plot_information(result, plt, output):
    """按同一54列基展示条件数、最弱方向和每样本D信息的对照图。"""
    rows = [next(r for r in result["dynamic"] if r["name"] == name) for name in PRIMARY]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.7))
    x = np.arange(len(rows))
    scopes = [
        ("planned_matched", "规划参考"),
        ("measured_drive_sg21", "驱动速度SG21"),
        ("measured_position_derivatives", "位置微分复核"),
    ]
    specs = [
        ("condition", "固定尺度条件数 · 越低越好"),
        ("minimum_information_relative", "最弱方向相对信息 · 越高越好"),
        ("D_information_relative", "每样本D信息 · 越高越好"),
    ]
    for ax, (key, title) in zip(axes, specs):
        for i, (scope, label) in enumerate(scopes):
            ax.bar(
                x + (i - 1) * 0.24,
                [r["information"][scope][key] for r in rows],
                0.24,
                label=label,
            )
        ax.set_xticks(x, [LABELS[r["name"]] for r in rows])
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.2)
    axes[0].legend(fontsize=8, frameon=False)
    fig.suptitle("同一54列基、同一列尺度、样本数归一化：条件数改善不等于总信息增加")
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(output / "information_comparison.png")
    plt.close(fig)


def plot_phase_space(previews, plt, output):
    """输出J2/J3角度及J3位置—速度轨迹图，区分包络范围与实际轨迹。"""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for name in PRIMARY:
        _, q, v = previews[name]
        axes[0].plot(
            np.rad2deg(q[:, 1]),
            np.rad2deg(q[:, 2]),
            lw=0.6,
            alpha=0.7,
            label=LABELS[name],
        )
        axes[1].plot(
            v[:, 2], np.rad2deg(q[:, 2]), lw=0.6, alpha=0.7, label=LABELS[name]
        )
    axes[0].set(
        xlabel="J2角度 / °", ylabel="J3角度 / °", title="角度范围内也只是有限轨迹"
    )
    axes[1].set(
        xlabel="J3速度 / rad/s",
        ylabel="J3角度 / °",
        title="位置—速度覆盖：相同范围不代表相同激励",
    )
    for ax in axes:
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(output / "phase_space.png")
    plt.close(fig)


def build_report_lines(result):
    """将既有指标组织为中文解释报告，不重算或修改数值口径。

    保留历史模型来源、低速诊断假设、信息代理局限与独立验证边界。"""
    primary = [
        next(r for r in result["dynamic"] if r["name"] == name) for name in PRIMARY
    ]
    latest = primary[-1]
    lines = [
        "# PiPER 激励轨迹范围、速度与信息量审计",
        "",
        "范围：8条既有实机激励轨迹、2份冻结优化候选库、6次单轴低速摩擦采集。仅离线读取，不改变轨迹、模型、控制参数或训练/验证划分。",
        "",
        "## 动作范围：20%到底指什么",
        "",
        "主口径为每轴实测最大角度减最小角度，除以同次固件记录的关节总行程；六轴均值只作为便于沟通的摘要。不能解释为六维构型空间、末端可达空间或无碰撞空间的体积覆盖率。",
        "",
        "| 实机轨迹 | J1 | J2 | J3 | J4 | J5 | J6 | 六轴均值 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in primary:
        c = row["coverage"]
        lines.append(
            "| "
            + LABELS[row["name"]]
            + " | "
            + " | ".join(f"{x * 100:.1f}%" for x in c["range_fraction"])
            + f" | {100 * c['mean_marginal_fraction']:.1f}% |"
        )
    lines += [
        "",
        f"优化v2实测角度范围（度）：{np.round(latest['coverage']['min_deg'], 2).tolist()} 到 {np.round(latest['coverage']['max_deg'], 2).tolist()}。",
        f"8条轨迹合并后的逐轴包络比例为：{np.round(100 * np.array(result['joint_union_coverage']['range_fraction']), 1).tolist()}%。合并包络也不意味着内部被均匀访问。",
        "",
        "原生成器的amplitude_fraction=0.6限制的是所有傅里叶系数绝对值之和；各谐波峰值不会同时出现。旧轨迹还乘0.5执行幅度。优化v2的scale=1也只表示系数预算未再缩小，不能解读为覆盖100%关节行程。",
        "",
        "## 速度与低速停留",
        "",
        "主统计使用实测位置插值到10ms网格、SG21三阶位置微分；SG41微分和驱动速度保留为对照。数据缺口及最大51点滤波支撑全部剔除，动态统计只取包络外稳态窗口。",
        "低速诊断区为0.002≤|v|<0.02 rad/s，不把静止/近零驻留算低速运动；稳定低速另要求|a|≤0.05 rad/s²，并检查连续至少0.25s。阈值是审计口径，不是已验证的Stribeck速度常数。>=0.2 rad/s只表示本批数据的较高速段，不能代表覆盖到机械臂3 rad/s的固件速度上限。",
        "",
        "| 轨迹 | 指标 | J1 | J2 | J3 | J4 | J5 | J6 |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in primary:
        speeds = row["speed_position_sg21"]["joints"]
        for key, label, multiplier in [
            ("peak_rad_s", "峰值rad/s", 1),
            ("low_moving_fraction", "低速时间%", 100),
            ("above_0p2_fraction", ">=0.2时间%", 100),
        ]:
            lines.append(
                "| "
                + LABELS[row["name"]]
                + " | "
                + label
                + " | "
                + " | ".join(f"{s[key] * multiplier:.2f}" for s in speeds)
                + " |"
            )
    lines += [
        "",
        "优化v2稳定低速连续片段（上述阈值）：",
        "",
        "| 关节 | 总稳定低速秒数 | 最长片段秒数 | >=0.25秒片段数 |",
        "|---|---:|---:|---:|",
    ]
    for s in latest["speed_position_sg21"]["joints"]:
        v = s["stable_low_residence"]
        lines.append(
            f"| J{s['joint']} | {v['seconds']:.2f} | {v['longest_visit_s']:.3f} | {v['visits_at_least_0p25s']} |"
        )
    lines += [
        "",
        "## 条件数与信息量",
        "",
        "采用修订001的共同36列惯性基＋18列粘性/平滑库仑/偏置，全部比较使用同一固定列RMS尺度。计算SVD(A/scale/sqrt(标量行数))；报告条件数、最小奇异值平方的相对值、exp(2mean(logσ))的相对值。后两者分别表示最弱方向与D准则的等噪声设计代理；并未标定有色测量噪声协方差。",
        "相对信息量均以同类信号来源的原快轨迹为1；每样本归一化使慢轨迹不会仅因记录时间翻倍而占优。总D代理另见JSON，不能把100Hz相关样本全部当独立观测。",
        "",
        "| 轨迹 | 数据来源 | 秩 | 条件数 | 最弱方向相对信息 | 每样本D相对信息 |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for row in primary:
        for scope, name in [
            ("planned_matched", "规划"),
            ("measured_drive_sg21", "驱动速度SG21"),
            ("measured_position_derivatives", "位置微分"),
        ]:
            v = row["information"][scope]
            lines.append(
                f"| {LABELS[row['name']]} | {name} | {v['rank']} | {v['condition']:.2f} | {v['minimum_information_relative']:.3f} | {v['D_information_relative']:.3f} |"
            )
    lines += [
        "",
        "所有Pinocchio信息矩阵均以存档MuJoCo回归器抽样复核；误差与输入SHA256见metrics.json。完整78列存在结构不可辨识方向，不能直接拿其无穷条件数评价轨迹。",
        "",
        "## 非线性Stribeck可辨识性",
        "",
        "线性54列满秩并不验证Stribeck。额外检查模型 [Fc+ΔFs exp(-(v/vs)^2)]tanh(v/w)+fv*v+offset 对ΔFs与log(vs)的局部敏感度，投影掉粘性、库仑和偏置干扰方向。扫描vs=0.003、0.005、0.01、0.02、0.05、0.1 rad/s，分别使用旧模型w=0.02及低速模型w=0.002；这些值为假设网格，不能当实测参数。参考ΔFs=0.1Nm；若真实ΔFs=0，vs本来就不可辨识。详细逐轴结果见stribeck_sensitivity.csv。",
        "",
        "经过换向零速附近，只证明存在低速瞬间；辨识低速摩擦还需足够长的正反向低加速度段，区分惯性、重力、摩擦、回差与噪声。",
        "",
        "## 已有六轴摩擦采集提供的补充",
        "",
        "本机已存在0.005、0.01、0.02、0.05、0.1、0.2 rad/s档位的正反/返回采集；统计排除hold相位，只用有效移动阶段。",
        "",
        "| 关节 | 低速运动比例% | 最长稳定低速s | >=0.25s稳定低速片段数 |",
        "|---|---:|---:|---:|",
    ]
    for row in result["friction"]:
        s = row["speed"]
        lines.append(
            f"| J{row['joint']} | {s['low_moving_fraction'] * 100:.1f} | {s['stable_low_residence']['longest_visit_s']:.2f} | {s['stable_low_residence']['visits_at_least_0p25s']} |"
        )
    lines += [
        "",
        "这些单轴数据是动态多正弦的补充，并不能替代多轴动态激励。当前最终摩擦拟合主要为粘性＋tanh库仑＋偏置，拟合了它不等于验证了Stribeck下降分支；更低于0.005 rad/s的非零速度区域仍需结合敏感度、量化及留出验证判断。",
        "",
        "## 优化目标、预算与历史模型边界",
        "",
    ]
    for bank in result["candidate_banks"]:
        p = bank["provenance"]
        lines += [
            f"### {bank['file'].split('/')[-1]}",
            "",
            f"随机种子{p['seed']}，每组幅度/速度随机候选{p['random_candidates_per_scale_pair']}，幅度网格{p['amplitudes']}，速度网格{p['speed_scales']}；局部每起点迭代预算{p['iterations_per_start']}，记录总评估次数{p['total_evaluations']}。",
            f"局部优化状态：{[x['success'] for x in bank['optimization_log']]}；达到预算与数学收敛应区别，不宣称全局最优。",
            "",
        ]
    lines += [
        "目标是log(kappa/kappa0)−0.5log(smin/smin0)−0.5(logD−logD0)，同时受位置、速度、加速度、jerk及简化几何限制。没有把逐轴覆盖、低速连续停留或Stribeck非线性敏感度直接纳入目标。v2设计还明确允许D信息相对基线降到0.25，接受信息与可执行性的折中。",
        "冻结优化库保留历史X模型设计来源，后在普通PiPER上重新评分；重新评分不是在普通PiPER上重新优化，当前源码也不能改写为原优化时已经使用现模型。相关模型指纹、历史状态均保留在JSON。",
        "",
        "## 评价与后续设计建议",
        "",
        "应同时报告：逐轴覆盖与二维相空间、速度档位及正反向停留、共同基条件数、D/E信息、Stribeck条件信息、峰值运动预算，以及独立验证误差。覆盖更大和条件数更低都不单独保证模型更准。",
        "建议将动态激励与低速摩擦视为互补实验：动态部分在明确使用姿态范围内选择多个中心姿态，优化共同基D/E信息并保留jerk/约束；低速部分明确安排非零对数速度档位、正反向近匀速平台与重复，覆盖待识别vs附近。最终按整条轨迹/速度档/姿态留出验证，不用训练指标自行宣布优胜。本次不生成、不自动执行新实机轨迹。",
        "",
        "理论参考：[Swevers等，Optimal robot excitation and identification](https://lirias.kuleuven.be/bitstream/123456789/166506/1/robot_identification.pdf)；[独立关节恒速摩擦实验示例](https://pmc.ncbi.nlm.nih.gov/articles/PMC6566985/)。",
        "",
        "复现：`PYTHONPATH=src:. python -m piper_dynamics.pipeline.trajectory_audit --output outputs/新的审计目录`。使用项目已有Python环境，输出目录有结果时拒绝覆盖。",
    ]
    return lines


def write_report(result, previews, output):
    """使用无界面绘图后端输出表格、图片、方法JSON和Markdown报告。

    output由上层选择新目录；本函数写入同名结果文件并在每张图保存后释放图对象。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "Noto Sans CJK JP",
            "axes.unicode_minus": False,
            "font.size": 10,
            "figure.dpi": 150,
        }
    )
    write_csv_files(result, output)
    plot_coverage(result, plt, output)
    plot_speed(result, plt, output)
    plot_information(result, plt, output)
    plot_phase_space(previews, plt, output)
    (output / "report.md").write_text("\n".join(build_report_lines(result)) + "\n")
    (output / "method.json").write_text(
        json.dumps(result["method"], ensure_ascii=False, indent=2)
    )
