# Offline dependency subset

This directory contains the unchanged dependency subset needed by the current
offline identification scripts: the original regressor and solver, PiPER X
model adapter and assets, measurement preprocessing, and CAN decoder.

`train_test/run_piper_x_identification.py` pins the model and algorithm source
hashes. The snapshot is retained separately from the root simulation files so
future edits to the root workflow do not change the archived analysis model.

The hardware acquisition/control application and its tests are not included.
Only `model_xml()` from `real_robot/simulation.py` is used by the offline model
adapter; the preview application's other entry points are outside this subset.
