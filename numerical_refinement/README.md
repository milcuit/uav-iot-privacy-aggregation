# Numerical check

This directory contains a check of the contact-time accounting used by the
recovery simulation. The check keeps the original routes, seeds, online states,
and selected terminal sets fixed.

It compares the original 0.5 s time-step implementation with a finer time
step and an event-driven calculation based on line-segment and coverage-circle
intersections. The latter releases the shared service capacity as soon as a
terminal finishes its transfer.

The check does not change the protocol model or the paper's parameter sweep.
Its output is useful for checking numerical sensitivity; it should not be
reported as an additional experiment.

Run it from the parent directory:

```bash
python audit_numerics.py
python diagnose_refinement.py
python plot_numerical_refinement.py
```
