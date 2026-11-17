# UAV mobility and threshold-recovery experiments

## About

This repository contains the implementation and experimental code for the paper:

**"Practical Collusion-Resistant Privacy-Preserving Aggregation for UAV-Assisted IoT"**

*Submitted to IEEE Transactions on Dependable and Secure Computing (TDSC), currently under review.*

## Overview

This repository contains the simulator, input data, and result files for the
UAV mobility evaluation in the paper. The recovery stage models the following
exchange: after aggregation, the CC selects `t` terminals, sends one ciphertext
polynomial to each selected terminal through the UAV, and collects one partial
decryption response from each terminal.

The code evaluates communication completion. It does not implement the
cryptographic algorithms, packet headers, computation time, explicit discovery
messages, or the UAV-to-CC backhaul.

## Running the experiment

Python 3.10 or later is recommended.

```bash
python -m venv .venv
# Linux/macOS: source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python run_all.py
```

`run_all.py` runs the collection stage, prepares its output, runs the recovery
stage, and regenerates the heatmap. On Windows, `run_all.ps1` is an equivalent
wrapper.

The run uses 200 paired trials. The random seed and other parameters are stored
in `collection_stage_data/experiment_config.json`.

## Files

- `mobility_collection_experiment.py`: data-report collection simulation.
- `prepare_collection_input.py`: converts collection output for the recovery
  stage.
- `mobility_two_stage.py`: post-aggregation recovery communication simulation.
- `plot_two_stage.py`: generates the paper heatmap.
- `mobility_protocol_recovery_results/summary.csv`: rates and Wilson intervals.
- `mobility_protocol_recovery_results/per_run.csv`: per-trial results.
- `collection_stage_data/` and `collection_stage_raw/`: collection inputs and
  outputs.
- `numerical_refinement/`: event-driven contact-accounting check.
- `fault_tolerance_mobility.tex`: the manuscript subsection used with the
  figure.

The included CSV and PNG files are the results used for the paper. Running the
scripts overwrites the corresponding generated files.

## Model

There are 100 terminals in a 500 m by 500 m field. Each terminal is marked
online independently with probability 0.9 at the start of a trial and keeps
that state for the trial. During collection, every online terminal that enters
the communication range is added to the set known to the CC. The CC selects
`t=60` terminals from this set. A trial with fewer than 60 known terminals
cannot enter the recovery stage.

The UAV flies at 100 m altitude on either a preplanned patrol route or a
random-waypoint route. The collection and recovery legs have the same length.
The link model uses distance-dependent air-to-ground path loss, a 20 dB SNR
threshold, 20 MHz physical bandwidth, and the goodput values in the
configuration. Each selected terminal receives one 6,912-byte polynomial and
returns one 6,912-byte response. Transfers resume at later contacts and share
the effective goodput budget. UAV-to-CC forwarding is reliable in this model.

The random-waypoint route is not optimized for coverage. Its results are used
to show how the recovery communication changes with the contact pattern; they
are not a comparison between cryptographic constructions.

The recovery script reads `threshold`, `polynomial_bytes`, and
`response_bytes` from the configuration when present. Otherwise it uses 60,
6,912, and 6,912 bytes, respectively.

## Numerical check

The optional numerical check compares the original 0.5 s time-step model with
finer and event-driven contact accounting. It keeps the selected terminal sets
fixed. Run it from the repository root with:

```bash
python audit_numerics.py
python diagnose_refinement.py
python plot_numerical_refinement.py
```

This check is separate from the paper's main result.

## License

MIT License. See [LICENSE](LICENSE) for details.
