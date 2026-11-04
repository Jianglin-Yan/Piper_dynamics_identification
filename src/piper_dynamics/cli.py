"""Command line interface: ``python -m piper_dynamics <command>``."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from .core.parameter_report import print_parameters
from .paths import RAW_INPUTS, TRAIN_INPUTS, VALIDATION_INPUTS
from .pipeline.standard_piper_pipeline import build_regressor, default_fit_path, fit
from .pipeline.trajectory_split import (
    load_split_rule,
    materialise_splits,
    raw_dataset_path,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m piper_dynamics",
        description="Offline dynamics identification for the standard PiPER arm.",
    )
    subcommands = parser.add_subparsers(dest="command", required=True)

    split = subcommands.add_parser(
        "split", help="copy the train/validation acquisitions into inputs/"
    )
    split.add_argument("--raw-root", type=Path, default=RAW_INPUTS)
    split.add_argument("--train-out", type=Path, default=TRAIN_INPUTS)
    split.add_argument("--validation-out", type=Path, default=VALIDATION_INPUTS)

    show = subcommands.add_parser(
        "show", help="print the fitted parameters readably"
    )
    show.add_argument("path", nargs="?", type=Path)

    regressor = subcommands.add_parser(
        "regressor", help="write the 60-column rigid-body regressor"
    )
    regressor.add_argument("dataset", nargs="?", type=Path)
    regressor.add_argument("--out", type=Path)

    fitted = subcommands.add_parser(
        "fit", help="fit on the training acquisition and evaluate on validation"
    )
    fitted.add_argument("dataset", nargs="?", type=Path, help="training npz")
    fitted.add_argument("--validation", type=Path, help="held-out npz")
    fitted.add_argument("--out", type=Path)
    fitted.add_argument("--rank-tolerance", type=float, default=1e-8)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "split":
        materialise_splits(args.raw_root, args.train_out, args.validation_out)
    elif args.command == "show":
        print_parameters((args.path or default_fit_path()).resolve())
    elif args.command == "regressor":
        dataset = args.dataset or raw_dataset_path(load_split_rule()["train"])
        build_regressor(dataset, args.out)
    else:
        fit(args.dataset, args.validation, args.out, args.rank_tolerance)
    return 0
