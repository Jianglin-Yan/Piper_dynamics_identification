"""Explicit inputs, revision and output paths for the existing friction analyses."""

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

from piper_dynamics.paths import IDENTIFICATION_ARCHIVE, INPUTS, OUTPUTS, RUNS


@dataclass(frozen=True)
class FrictionPaths:
    run_root: Path
    revision: Path
    source: Path
    output: Path
    runs: dict[int, str]


def parse_paths(description, argv=None):
    """Select existing acquisitions/revision; never modify the experiment archive."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--runs-file", type=Path, default=INPUTS / "friction_runs.json")
    parser.add_argument("--run-root", type=Path, default=RUNS)
    parser.add_argument("--archive", type=Path, default=IDENTIFICATION_ARCHIVE)
    parser.add_argument("--revision", type=Path)
    parser.add_argument("--source", type=Path, default=OUTPUTS / "friction_fit")
    parser.add_argument("--out", type=Path, default=OUTPUTS / "friction_fit")
    args = parser.parse_args(argv)
    revision = args.revision
    if revision is None:
        active = json.loads((args.archive / "active_revision.json").read_text())
        if active.get("model") != "piper":
            raise ValueError("The selected revision must belong to standard PiPER")
        revision = args.archive / active["revision_directory"]
    runs = json.loads(args.runs_file.read_text())["runs"]
    args.out.mkdir(parents=True, exist_ok=True)
    return FrictionPaths(
        args.run_root,
        revision,
        args.source,
        args.out,
        {int(joint): name for joint, name in runs.items()},
    )
