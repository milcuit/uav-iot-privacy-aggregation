#!/usr/bin/env python3
"""Run the primary collection, recovery, and plotting pipeline."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def run(*args: str) -> None:
    command = [sys.executable, *args]
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def main() -> None:
    run("mobility_collection_experiment.py", "--output-dir", "collection_stage_raw")
    run(
        "prepare_collection_input.py",
        "--source",
        "collection_stage_raw",
        "--output",
        "collection_stage_data",
    )
    run("mobility_two_stage.py")
    run("plot_two_stage.py")
    print("Primary artifact reproduction completed.", flush=True)


if __name__ == "__main__":
    main()
