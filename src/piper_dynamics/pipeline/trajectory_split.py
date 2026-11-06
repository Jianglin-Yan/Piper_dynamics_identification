"""Trajectory-level train/validation split and materialisation.

``inputs/raw/standard_piper/splits.json`` names one whole real acquisition for
fitting and a different whole acquisition for held-out validation. The split
command copies both into ``inputs/train`` and ``inputs/validation`` so that the
fitting code never touches the validation file.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..core.dataset_io import load_archive
from ..paths import RAW_INPUTS, TRAIN_INPUTS, VALIDATION_INPUTS
from ..utils.hashing import sha256

SPLIT_RULE = RAW_INPUTS / "splits.json"
SPLIT_VARIANT = "sg41"


def load_split_rule(path: Path = SPLIT_RULE) -> dict:
    """Read and validate the trajectory-level split rule."""
    rule = json.loads(path.read_text())
    for key in ("train", "validation"):
        if not isinstance(rule.get(key), str) or not rule[key]:
            raise ValueError(f"{path}: '{key}' must name exactly one trajectory label")
    if rule["train"] == rule["validation"]:
        raise ValueError(f"{path}: training and validation trajectories must differ")
    rule.setdefault("variant", SPLIT_VARIANT)
    return rule


def raw_dataset_path(label: str, variant: str = SPLIT_VARIANT) -> Path:
    """Path of one archived trajectory variant under ``inputs/raw``."""
    return RAW_INPUTS / label / f"{variant}.npz"


def default_dataset_paths(rule: dict | None = None) -> tuple[Path, Path]:
    """Training and validation files under ``inputs/train`` and ``inputs/validation``."""
    rule = rule or load_split_rule()
    variant = rule["variant"]
    return (
        TRAIN_INPUTS / f"{rule['train']}_{variant}.npz",
        VALIDATION_INPUTS / f"{rule['validation']}_{variant}.npz",
    )


def output_stem(dataset: Path) -> str:
    """Stable output prefix for a dataset path."""
    if dataset.parent.name in (TRAIN_INPUTS.name, VALIDATION_INPUTS.name):
        return dataset.stem
    return f"{dataset.parent.name}_{dataset.stem}"


def materialise_splits(
    raw_root: Path = RAW_INPUTS,
    train_dir: Path = TRAIN_INPUTS,
    validation_dir: Path = VALIDATION_INPUTS,
) -> list[Path]:
    """Copy the train/validation acquisitions declared by ``splits.json``."""
    rule_path = raw_root / "splits.json"
    rule = load_split_rule(rule_path)
    variant = rule["variant"]
    train_dir.mkdir(parents=True, exist_ok=True)
    validation_dir.mkdir(parents=True, exist_ok=True)
    roles = (
        (
            "train",
            rule["train"],
            train_dir,
            "Training data: one whole real acquisition used for fitting.",
        ),
        (
            "validation",
            rule["validation"],
            validation_dir,
            "Held-out data: an independent real acquisition used only for validation.",
        ),
    )
    written: list[Path] = []
    for _, label, directory, description in roles:
        expected = f"{label}_{variant}.npz"
        for stale in directory.glob(f"*_{variant}.npz"):
            if stale.name != expected:
                stale.unlink()
                print(f"removed stale {stale}")
        source = raw_root / label / f"{variant}.npz"
        data = load_archive(source)
        out = directory / expected
        np.savez_compressed(out, **data)
        manifest = {
            "description": description,
            "rule": rule.get("rule"),
            "split_rule_file": str(rule_path.relative_to(raw_root.parents[1])),
            "split_rule_sha256": sha256(rule_path),
            "variant": variant,
            "trajectory": label,
            "file": out.name,
            "sha256": sha256(out),
            "source": str(source.relative_to(raw_root.parents[1])),
            "source_sha256": sha256(source),
            "n_samples": int(len(data["cycle"])),
            "cycles": sorted(int(c) for c in np.unique(data["cycle"])),
        }
        manifest_path = directory / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
        written.extend([out, manifest_path])
        print(f"wrote {out} ({manifest['n_samples']} samples from {label})")
    return written
