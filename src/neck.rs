//! 3-DOF serial neck (Z-Y-X Euler gimbal).
//!
//! `fk(yaw, pitch, roll)` composes `Rz(yaw) · Ry(pitch) · Rx(roll)`.
//! `ik(R)` extracts those three angles back. Sign convention follows the
//! Dreambo torso: `+pitch` = chin up, `+yaw` = look left, `+roll` = head
//! tilts toward the right shoulder.

use nalgebra::{Matrix3, Rotation3, Unit, Vector3};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};

const PITCH_GIMBAL_LOCK_EPS: f64 = 1e-6;

#[gen_stub_pyclass]
#[pyclass(frozen, module = "dreambo_torso_kinematics")]
pub struct DreamboNeckKinematics;

#[gen_stub_pymethods]
#[pymethods]
impl DreamboNeckKinematics {
    #[new]
    fn new() -> Self {
        Self
    }

    /// Forward kinematics. `joints = [yaw, pitch, roll]` in radians.
    /// Returns the 3×3 rotation matrix of the head frame in the body frame.
    fn fk(&self, joints: [f64; 3]) -> [[f64; 3]; 3] {
        let r = compose_zyx(joints[0], joints[1], joints[2]);
        mat3_to_array(&r.into_inner())
    }

    /// Inverse kinematics. `r` is a 3×3 rotation matrix (row-major).
    /// Returns `(yaw, pitch, roll)` in radians.
    ///
    /// Raises `ValueError` near gimbal lock (|pitch| ≈ π/2). The neck's
    /// pitch range is well under that limit in practice.
    fn ik(&self, r: [[f64; 3]; 3]) -> PyResult<(f64, f64, f64)> {
        decompose_zyx(&array_to_mat3(&r))
            .ok_or_else(|| PyValueError::new_err("Gimbal lock: pitch is too close to ±π/2."))
    }
}

fn compose_zyx(yaw: f64, pitch: f64, roll: f64) -> Rotation3<f64> {
    let rz = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::z()), yaw);
    let ry = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::y()), pitch);
    let rx = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::x()), roll);
    rz * ry * rx
}

fn decompose_zyx(r: &Matrix3<f64>) -> Option<(f64, f64, f64)> {
    // R = Rz · Ry · Rx
    // R[2,0] = -sin(pitch), R[2,1] = cos(pitch)·sin(roll), R[2,2] = cos(pitch)·cos(roll)
    // R[1,0] = sin(yaw)·cos(pitch), R[0,0] = cos(yaw)·cos(pitch)
    let sin_pitch = -r[(2, 0)];
    if (1.0 - sin_pitch.abs()) < PITCH_GIMBAL_LOCK_EPS {
        return None;
    }
    let pitch = sin_pitch.asin();
    let cos_pitch = pitch.cos();
    let roll = r[(2, 1)].atan2(r[(2, 2)]);
    let yaw = r[(1, 0)].atan2(r[(0, 0)]);
    debug_assert!(cos_pitch.abs() > 0.0);
    Some((yaw, pitch, roll))
}

fn mat3_to_array(m: &Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
        [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
        [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
    ]
}

fn array_to_mat3(a: &[[f64; 3]; 3]) -> Matrix3<f64> {
    Matrix3::new(
        a[0][0], a[0][1], a[0][2],
        a[1][0], a[1][1], a[1][2],
        a[2][0], a[2][1], a[2][2],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fk_identity_at_zero() {
        let neck = DreamboNeckKinematics;
        let r = neck.fk([0.0, 0.0, 0.0]);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((r[i][j] - expected).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn ik_round_trip() {
        let neck = DreamboNeckKinematics;
        // Sample (yaw, pitch, roll) inside the Dreambo neck workspace.
        let samples = [
            (0.1, 0.2, 0.0),
            (-0.3, 0.4, 0.2),
            (0.5, 0.0, -0.3),
            (-0.4, 0.15, 0.05),
        ];
        for &(y, p, r) in &samples {
            let m = neck.fk([y, p, r]);
            let (y2, p2, r2) = neck.ik(m).unwrap();
            assert!((y - y2).abs() < 1e-9, "yaw mismatch: {} vs {}", y, y2);
            assert!((p - p2).abs() < 1e-9, "pitch mismatch: {} vs {}", p, p2);
            assert!((r - r2).abs() < 1e-9, "roll mismatch: {} vs {}", r, r2);
        }
    }

    #[test]
    fn ik_rejects_gimbal_lock() {
        let neck = DreamboNeckKinematics;
        let m = neck.fk([0.0, std::f64::consts::FRAC_PI_2, 0.0]);
        assert!(neck.ik(m).is_err());
    }
}