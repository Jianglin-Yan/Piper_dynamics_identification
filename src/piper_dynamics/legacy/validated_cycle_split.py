"""Validate the archived cycle split against the measurements being fitted."""

from pathlib import Path

import numpy as np


def load_validated_split(path: Path, signals: dict, valid_indices: np.ndarray):
    with np.load(path) as split:
        source = split["aligned_signal_index"].astype(int)
        is_train = split["split_is_train"].astype(bool)
        comparisons = {
            "t_test_s": "t",
            "q_meas_rad": "q",
            "qd_meas_rad_s": "qd",
            "qdd_est_rad_s2": "qdd",
            "current_A": "current_A",
        }
        if len(source) != len(is_train) or len(np.unique(source)) != len(source):
            raise ValueError("Cycle split has duplicate indices or mismatched labels")
        if not np.all(np.isin(source, valid_indices)):
            raise ValueError("Cycle split references samples outside the valid measurement set")
        for split_key, signal_key in comparisons.items():
            if not np.array_equal(split[split_key], signals[signal_key][source]):
                raise ValueError(
                    f"Cycle split {split_key} differs from this run or filter window; "
                    "generate a matching split before fitting"
                )

    row_map = {int(index): row for row, index in enumerate(valid_indices)}
    train_rows = np.asarray([row_map[int(i)] for i in source[is_train]], dtype=int)
    validation_rows = np.asarray([row_map[int(i)] for i in source[~is_train]], dtype=int)
    if np.intersect1d(train_rows, validation_rows).size:
        raise ValueError("Training and validation rows overlap")
    return train_rows, validation_rows
