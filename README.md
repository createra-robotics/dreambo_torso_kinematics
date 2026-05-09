# Dreambo Torso Kinematics

Analytical Inverse Kinematics, Numerical Forward Kinematics

## Summary

A Rust implementation (with Python bindings via PyO3) of the kinematics for the Dreambo torso — a 6-branch parallel mechanism where each branch is a motor-driven arm linked to the moving platform by a fixed-length rod.

- **Inverse kinematics** is solved analytically per branch: given a target platform pose, each motor angle is computed in closed form by intersecting the motor arm circle with the rod sphere centered on the corresponding platform anchor (the `solution` flag selects which of the two roots is used).
- **Forward kinematics** is solved numerically: starting from the last known platform pose, the solver iterates a damped Newton step using the 3×6 platform-to-anchor Jacobian (Varignon's formula for the angular part) plus a halving line search until the rod-length residuals fall below tolerance.
- **Body yaw** is supported as an optional extra DOF on top of the 6-DOF parallel platform, with safety-clamped variants (`inverse_kinematics_safe`) that bound relative and absolute yaw to the mechanical limits.

The whole solver runs without dynamic allocation in the hot loop and is exposed to Python as `DreamboTorsoKinematics` from the `dreambo_torso_kinematics` module. Geometry per branch (platform anchor, world-to-motor transform, solution branch) is configured at runtime via `add_branch`; see `motors.json` for the production Dreambo configuration.

---

## To install locally

```bash
pip install maturin
```

## To build the wheel
```bash
pip install -e . --verbose
```

## To install the wheel

```bash
cd `target/wheels`
pip install dreambo_torso_kinematics...
```

---

## Local Development

```bash
uv sync
uv pip install maturin pytest


uv run maturin develop --release
env -u CONDA_PREFIX uv run --no-sync python test.py
```

