//! Analytical kinematics for the Dreambo torso.
//!
//! Two PyO3 classes:
//!
//! - [`DreamboArmKinematics`] — spherical 5-bar (Olaf-style) shoulder.
//!   2-DOF input (θ_A yaw motor, θ_B pitch motor) → 3-DOF end-effector
//!   rotation. Closed-form FK and IK.
//! - [`DreamboNeckKinematics`] — 3-DOF serial gimbal. ZYX Euler.

pub mod arm;
pub mod neck;

use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;

pub use arm::DreamboArmKinematics;
pub use neck::DreamboNeckKinematics;

#[pyo3::pymodule]
fn dreambo_torso_kinematics(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<DreamboArmKinematics>()?;
    m.add_class::<DreamboNeckKinematics>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);
