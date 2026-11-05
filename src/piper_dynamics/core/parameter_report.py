"""Human-readable parameter tables, CSV export and Markdown reports."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .parameter_schema import FRICTION_NAMES, FRICTION_UNITS, LINK_NAMES, LINK_UNITS
from .rigid_body_regressor import describe_theta


def parameter_table(theta: np.ndarray, nominal: np.ndarray) -> list[dict]:
    """Readable rows: link mass/COM/inertia about the COM, plus friction terms."""
    identified = describe_theta(np.asarray(theta[:60]).reshape(6, 10), LINK_NAMES)
    nominal_links = describe_theta(np.asarray(nominal[:60]).reshape(6, 10), LINK_NAMES)
    rows: list[dict] = []
    for index, (fit_row, prior_row) in enumerate(zip(identified, nominal_links)):
        for key, unit in LINK_UNITS.items():
            rows.append(
                {
                    "group": "link",
                    "name": f"{LINK_NAMES[index]}.{key}",
                    "unit": unit,
                    "nominal": float(prior_row[key]),
                    "identified": float(fit_row[key]),
                    "delta": float(fit_row[key] - prior_row[key]),
                }
            )
    for joint in range(6):
        for offset, key in enumerate(FRICTION_NAMES):
            index = 60 + 3 * joint + offset
            rows.append(
                {
                    "group": "joint",
                    "name": f"joint{joint + 1}.{key}",
                    "unit": FRICTION_UNITS[key],
                    "nominal": float(nominal[index]),
                    "identified": float(theta[index]),
                    "delta": float(theta[index] - nominal[index]),
                }
            )
    return rows


def write_parameters_csv(path: Path, rows: list[dict]) -> None:
    """Write the parameter table as a long-format CSV."""
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["group", "name", "unit", "nominal", "identified", "delta"])
        for row in rows:
            writer.writerow(
                [
                    row["group"],
                    row["name"],
                    row["unit"],
                    f"{row['nominal']:.12g}",
                    f"{row['identified']:.12g}",
                    f"{row['delta']:.12g}",
                ]
            )


def physical_consistency_notes(rows: list[dict]) -> list[str]:
    """Flag non-physical masses and inertia tensors in the unconstrained fit."""
    by_link: dict[str, dict[str, float]] = {}
    for row in rows:
        if row["group"] == "link":
            link, key = row["name"].split(".", 1)
            by_link.setdefault(link, {})[key] = row["identified"]
    notes = []
    for link, value in by_link.items():
        mass = value["mass"]
        inertia = np.array(
            [
                [value["Ixx"], value["Ixy"], value["Ixz"]],
                [value["Ixy"], value["Iyy"], value["Iyz"]],
                [value["Ixz"], value["Iyz"], value["Izz"]],
            ]
        )
        smallest = float(np.linalg.eigvalsh(inertia).min())
        problems = []
        if mass <= 0:
            problems.append(f"质量 {mass:.4g} ≤ 0")
        if smallest <= 0:
            problems.append(f"I_com 最小特征值 {smallest:.4g} ≤ 0")
        if problems:
            notes.append(f"- {link}：" + "，".join(problems))
    return notes


def write_markdown_report(path: Path, report: dict, rows: list[dict]) -> None:
    """Write a Chinese Markdown report with metrics, parameters and warnings."""
    train, held_out = report["train"], report["held_out"]
    absolute_rmse = np.sqrt(
        held_out["squared_error_sum_Nm2"] / max(6 * report["validation_samples"], 1)
    )
    lines = [
        f"# PiPER 辨识结果：{report['split']['train']} → {report['split']['validation']}",
        "",
        f"- 机器人：{report['robot']}",
        f"- 方法：{report['method']}",
        f"- 训练数据：`{report['train_dataset']}`（{report['train_samples']} 样本）",
        f"- 验证数据：`{report['validation_dataset']}`（{report['validation_samples']} 样本）",
        f"- 保留秩：{report['rank']} / 78（相对阈值 {report['rank_tolerance']:g}）",
        f"- 力矩标定：{'是' if report['torque_calibrated'] else '否（target_Nm 来自 SDK 电流系数）'}",
        "",
        "## 预测误差",
        "",
        "| 关节 | 训练 RMSE (N·m) | 训练 RMSE/RMS | 验证 RMSE (N·m) | 验证 RMSE/RMS | 验证 R² |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for joint in range(6):
        lines.append(
            f"| {joint + 1} | {train['rmse_Nm'][joint]:.3f} | "
            f"{100 * train['rmse_over_target_rms'][joint]:.1f}% | "
            f"{held_out['rmse_Nm'][joint]:.3f} | "
            f"{100 * held_out['rmse_over_target_rms'][joint]:.1f}% | "
            f"{held_out['r2_per_joint'][joint]:.3f} |"
        )
    lines += [
        "",
        f"全局相对误差：训练 {100 * train['relative_error_norm_ratio']:.2f}%，"
        f"验证 **{100 * held_out['relative_error_norm_ratio']:.2f}%**；"
        f"验证绝对 RMSE {absolute_rmse:.3f} N·m。",
        "",
        "## 连杆参数",
        "",
        "质量 kg，质心 m，惯量 kg·m²（绕质心）。辨识值是力矩拟合候选，不是唯一可辨识的物理真值。",
        "",
        "| 参数 | 单位 | 名义值 | 辨识值 | 差值 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        if row["group"] == "link":
            lines.append(
                f"| {row['name']} | {row['unit']} | {row['nominal']:.6g} | "
                f"{row['identified']:.6g} | {row['delta']:.6g} |"
            )
    lines += [
        "",
        "## 摩擦项",
        "",
        "| 参数 | 单位 | 名义值 | 辨识值 | 差值 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        if row["group"] == "joint":
            lines.append(
                f"| {row['name']} | {row['unit']} | {row['nominal']:.6g} | "
                f"{row['identified']:.6g} | {row['delta']:.6g} |"
            )
    lines += ["", "## 物理一致性检查", ""]
    notes = physical_consistency_notes(rows)
    if notes:
        lines += [
            "以下连杆参数不满足基本物理约束（质量为正、惯量正定）：",
            "",
            *notes,
            "",
        ]
    else:
        lines += ["所有连杆质量与惯量张量均满足基本物理约束。", ""]
    lines += [
        "> 截断 SVD 只保证在可辨识方向上最小化力矩残差，不保证逐连杆参数物理可行；"
        "出现负惯量时只能把 `theta` 当作力矩预测参数，不能当 CAD 真值。",
        "> 摩擦项是拟合用的经验项，不是物理摩擦系数真值；不可辨识方向保留名义先验。",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_parameters(path: Path) -> None:
    """Print the fitted parameters of a saved fit without rebuilding the regressor."""
    with np.load(path) as archive:
        theta = archive["theta"]
        nominal = archive["nominal_theta"]
        rank = int(archive["rank"])
    rows = parameter_table(theta, nominal)
    print(f"{path}\nrank = {rank}/78\n")
    print(f"{'parameter':30s}{'unit':16s}{'nominal':>14s}{'identified':>14s}{'delta':>14s}")
    for row in rows:
        print(
            f"{row['name']:30s}{row['unit']:16s}{row['nominal']:14.6g}"
            f"{row['identified']:14.6g}{row['delta']:14.6g}"
        )
    report_path = path.with_suffix(".json")
    if report_path.exists():
        report = json.loads(report_path.read_text())
        print(
            "\nrelative error: train %.2f%%, held-out %.2f%%"
            % (
                100 * report["train"]["relative_error_norm_ratio"],
                100 * report["held_out"]["relative_error_norm_ratio"],
            )
        )
    print("\nnote: identified values are torque-prediction candidates, not physical truth.")
