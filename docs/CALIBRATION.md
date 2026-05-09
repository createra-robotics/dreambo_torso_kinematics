# Calibration: producing `motors.json`

`motors.json` is the **mechanical-geometry calibration file** for one specific Dreambo torso. It captures, per branch, where the rod's platform-side ball joint sits and where each motor is mounted, so the kinematics solver can convert between platform pose and the six motor angles. The file is loaded at runtime — it is **not** baked into the Rust binary — so a different physical unit just needs a different `motors.json`.

This document describes the schema, the frame conventions the solver assumes, and the recommended workflow for generating a new file from CAD or from a parametric model.

## File schema

```jsonc
[
  {
    "branch_position": [x, y, z],      // rod's platform-side ball joint,
                                       // expressed in the PLATFORM frame
    "T_motor_world":   [[...], ...],   // 4x4 homogeneous transform: world → motor
                                       // (i.e. world coordinates expressed in
                                       //  the motor frame). The Rust and Python
                                       //  loaders both call .inverse() on this
                                       //  before passing it to `add_branch`,
                                       //  which expects T_world_motor.
    "solution": 0 | 1                  // selects which root of the analytical
                                       // IK quadratic to use; tied to how the
                                       // physical arm is assembled.
  },
  // ... 6 entries total, one per branch
]
```

The Rust loader does (see `src/lib.rs` test loader):

```rust
fn load_one(motor: Motor, kinematics: &mut Kinematics) {
    // ... build `branch_position` and `t_motor_world` from the JSON fields ...
    let solution = if motor.solution != 0.0 { 1.0 } else { -1.0 };
    kinematics.add_branch(
        branch_position,
        t_motor_world.try_inverse().unwrap(),  // -> T_world_motor
        solution,
    );
}
```

The Python loader does the same with `np.linalg.inv(motor["T_motor_world"])`. So if you generate the file from a different starting frame convention, **invert before serializing** to keep the schema consistent.

## Frame conventions the solver assumes

These are not optional — `forward_kinematics` and `inverse_kinematics` in `src/lib.rs` are written against them. Get the conventions wrong and you will see plausible-looking-but-wrong joint angles.

### World frame
Arbitrary, but it must be the same frame in which `T_world_platform` is supplied at runtime. Conventionally placed at the base of the torso assembly.

### Platform frame
Attached to the moving platform. `branch_position` for each branch is expressed in this frame, so it should be the same frame your application uses when commanding `T_world_platform`. The platform origin is typically chosen at the geometric center of the six ball joints, but the solver does not require this — it only requires consistency.

### Motor frame
The closed-form IK and FK both compute the motor-arm tip as:

```
arm_motor = motor_arm_length * (cos θ, sin θ, 0)
```

Therefore, **per motor**:

- The motor's rotation axis must be `+Z` in the motor frame.
- The motor arm sweeps in the `X-Y` plane.
- `θ = 0` corresponds to the arm pointing along `+X`.

When you build `T_world_motor` (or its inverse `T_motor_world`), make sure the motor frame is laid out this way. Errors here typically manifest as joint angles that look "off by 90°" or that map sign-flipped to the physical motor.

### `solution` flag
The IK reduces each branch to:

```
y = 2 · py · rs + ε · √(...),  where ε ∈ {+1, -1}
joint_angle = wrap(2 · atan2(y, x))
```

`solution: 1` selects `ε = +1`, `solution: 0` selects `ε = -1`. Geometrically this corresponds to the two assembly orientations of the motor arm relative to the rod (arm "inside" vs "outside" the rod plane). Pick whichever matches the physical assembly.

## How to produce a new `motors.json`

Pick whichever workflow matches how the torso geometry is defined.

### Workflow A — Export from CAD (recommended for real hardware)

This is how the file currently checked in was produced (the trailing-digit numerical noise and the clean `(±√3/2, ±1/2)` rotation patterns are the give-away).

1. **Define a platform-frame mate connector** in CAD on the moving platform body, at the platform origin you intend to use.
2. **Define a world-frame mate connector** at the torso base, at the world origin you intend to use.
3. **For each of the 6 motors**, define a motor-frame mate connector that obeys the convention above (Z = rotation axis, +X = arm-zero direction).
4. **For each of the 6 ball joints on the platform**, identify the center point.
5. Use the CAD tool's API to read:
   - The transform of each motor mate connector relative to the world mate connector → that's `T_world_motor`. Invert it for the file (`T_motor_world`).
   - The position of each ball joint in the platform-frame mate connector → that's `branch_position`.
6. Pair each motor with the ball joint it is mechanically connected to (the rod connects motor _k_ to anchor _k_), and emit one JSON entry per pair.
7. Set `solution` to `0` for all entries initially; refine in step "Choosing the `solution` flag" below.

Tool-specific entry points:

- **Onshape**: REST API + FeatureScript. `Document → Workspace → MateConnector` transforms via the assemblies API.
- **SolidWorks**: `IComponent2.Transform2` for component poses; `IFeature.GetSpecificFeature2` for mate connectors and reference points.
- **FreeCAD / Fusion**: Python API, `Placement` objects on `App::Part`.

### Workflow B — Generate parametrically

If the geometry is defined by a small set of design parameters (motor radius, mounting height, anchor radius, arm length, …), you can skip CAD entirely and emit `motors.json` from a script. Sketch:

```python
import json, numpy as np

R_motor   = 0.071    # motor axis distance from center
Z_motor   = 0.035    # motor plane height in world frame
R_anchor  = 0.039    # ball-joint radius on platform
Z_anchor  = -0.0012  # ball-joint Z in platform frame
ARM_PAIR_SPLIT = np.deg2rad(10)  # angular offset between paired anchors

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

# Motor frame: origin on motor axis, +Z = rotation axis pointing toward center,
# +X = arm-zero direction. For a motor whose axis is horizontal and pointing
# radially inward, the world->motor rotation is "world Y aligned with motor X,
# world Z aligned with motor Y, world X aligned with motor Z" plus a Z-rotation
# by base_angle around the world Z.
R_axes = np.array([[0, 0, 1],
                   [1, 0, 0],
                   [0, 1, 0]])

motors = []
for i in range(6):
    base_angle = i * np.pi / 3
    pair_sign  = +1 if i % 2 == 0 else -1

    R_world_motor = Rz(base_angle) @ R_axes
    t_world_motor = Rz(base_angle) @ np.array([R_motor, 0, Z_motor])
    T_world_motor = np.eye(4)
    T_world_motor[:3, :3] = R_world_motor
    T_world_motor[:3,  3] = t_world_motor

    T_motor_world = np.linalg.inv(T_world_motor)  # schema stores world->motor

    anchor_angle    = base_angle + pair_sign * ARM_PAIR_SPLIT
    branch_position = (Rz(anchor_angle) @ np.array([R_anchor, 0, Z_anchor])).tolist()

    motors.append({
        "branch_position": branch_position,
        "T_motor_world":   T_motor_world.tolist(),
        "solution":        i % 2,
    })

with open("motors.json", "w") as f:
    json.dump(motors, f, indent=4)
```

This is a **template**, not the production parameters. Replace the constants with the real Dreambo design values, and adjust `R_axes` if the motor mounting orientation differs.

### Workflow C — Online calibration (optional refinement)

If machining tolerances are large enough that CAD data is not accurate enough, you can refine the geometry by least-squares fitting against measured (joint-angle, platform-pose) pairs:

1. Move the platform through a set of poses and record the encoder readings.
2. Use a separate platform-pose measurement (vision, dial indicator, etc.) for ground truth.
3. Optimize over a perturbation `(δT_world_motor_k, δbranch_position_k)` for each branch by minimizing the residual of `inverse_kinematics(T_world_platform_i) − measured_angles_i` summed over all poses `i`.
4. Bake the optimized values back into `motors.json`.

This is standard parallel-robot kinematic calibration and is overkill for most Dreambo units; CAD-derived data is typically sufficient.

## Choosing the `solution` flag

The flag selects between the two assembly orientations of the motor arm relative to the rod. Procedure:

1. Initialize all six entries with `solution: 0`.
2. Place the platform at its mechanical home pose (typically `T_world_platform = identity` plus a small `+Z` offset matching the assembled height; for the current Dreambo this is `Z = 0.177`).
3. Run `inverse_kinematics(T_world_platform)` and compare each motor angle against the physically measured arm angle at that home pose.
4. For any branch where the IK angle is sign-flipped or off by ~`π` from reality, flip its `solution` bit and recompute.
5. Verify by feeding the IK output through `forward_kinematics` and checking that it returns `T_world_platform` (this is what `test_ik_fk_consistency` does in `src/lib.rs`).

A correctly-calibrated `motors.json` should reproduce the values in `test_inverse_kinematics` (`src/lib.rs`):

```
[ 0.5469, -0.6912,  0.6291, -0.6291,  0.6912, -0.5469 ]
```

at the home pose, to within ~1e-6.

## Validating a new `motors.json`

Drop the file at the repo root (the loader uses CWD-relative paths) and run:

```bash
cargo test                      # runs the IK / FK / consistency tests
env -u CONDA_PREFIX uv run --no-sync python test.py
```

Both should print joint angles in the ±π range and IK→FK round-trip back to the input platform pose to ≤ 1e-4. If the round-trip fails, the most likely culprits are, in order: motor frame Z-axis convention, `solution` flag per branch, and platform-frame origin mismatch with the runtime caller.
