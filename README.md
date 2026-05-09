# Dreambo Torso Kinematics

Analytical Inverse Kinematics, Numerical Forward Kinematics

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

