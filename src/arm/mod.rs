//! `DreamboArmKinematics`: closed-form FK + IK for the Olaf-style
//! spherical 5-bar shoulder.

pub mod fk;
pub mod geometry;
pub mod ik;

use nalgebra::{Matrix3, Vector3};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};

pub use fk::{forward_kinematics, ArmPose, FkError};
pub use geometry::{Branch, SphericalLinkGeometry};
pub use ik::{inverse_kinematics, inverse_kinematics_from_direction, IkError};

#[gen_stub_pyclass]
#[pyclass(frozen, module = "dreambo_torso_kinematics")]
pub struct DreamboArmKinematics {
    geometry: SphericalLinkGeometry,
}

#[gen_stub_pymethods]
#[pymethods]
impl DreamboArmKinematics {
    /// Create an arm with explicit geometry parameters.
    ///
    /// - `arm_arc`: arc length (radians) of each input arm.
    /// - `coupler_arc`: arc length (radians) of each coupler segment.
    /// - `mirror`: when true, mirror the arm across the YZ plane (right arm).
    #[new]
    #[pyo3(signature = (arm_arc=0.7, coupler_arc=0.5, mirror=false))]
    fn new(arm_arc: f64, coupler_arc: f64, mirror: bool) -> Self {
        Self {
            geometry: SphericalLinkGeometry {
                arm_arc,
                coupler_arc,
                branch: Branch::Positive,
                mirror,
            },
        }
    }

    /// Build the default left-arm geometry.
    #[staticmethod]
    fn default_left() -> Self {
        Self {
            geometry: SphericalLinkGeometry::default_left(),
        }
    }

    /// Build the default right-arm geometry (mirror of left).
    #[staticmethod]
    fn default_right() -> Self {
        Self {
            geometry: SphericalLinkGeometry::default_right(),
        }
    }

    /// Build the arm from a JSON string matching `SphericalLinkGeometry`.
    #[staticmethod]
    fn from_json(json: &str) -> PyResult<Self> {
        let geometry: SphericalLinkGeometry = serde_json::from_str(json)
            .map_err(|e| PyValueError::new_err(format!("Invalid arms.json: {e}")))?;
        Ok(Self { geometry })
    }

    /// Forward kinematics: `(θ_A, θ_B)` → 3×3 rotation of the end-effector.
    /// Raises `RuntimeError` when the loop fails to close (target joint
    /// pair is outside the workspace).
    fn fk(&self, theta_a: f64, theta_b: f64) -> PyResult<[[f64; 3]; 3]> {
        let pose = forward_kinematics(&self.geometry, theta_a, theta_b)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok(mat3_to_array(&pose.r_world_ee))
    }

    /// Inverse kinematics: 3×3 rotation → `(θ_A, θ_B)`.
    /// Raises `RuntimeError` when the target orientation isn't reachable.
    fn ik(&self, r_target: [[f64; 3]; 3]) -> PyResult<(f64, f64)> {
        let r = array_to_mat3(&r_target);
        inverse_kinematics(&self.geometry, &r).map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    /// Inverse kinematics from just the end-effector pointing direction
    /// (3-vector, automatically normalized). When multiple `(θ_A, θ_B)`
    /// branches reach the same direction, `near` (an optional current
    /// joint pair) picks the closest one; otherwise the smallest-norm
    /// solution wins.
    #[pyo3(signature = (v3, near=None))]
    fn ik_from_direction(
        &self,
        v3: [f64; 3],
        near: Option<(f64, f64)>,
    ) -> PyResult<(f64, f64)> {
        let v = Vector3::new(v3[0], v3[1], v3[2]);
        inverse_kinematics_from_direction(&self.geometry, &v, near)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    /// Forward kinematics that returns just the end-effector pointing
    /// direction (the third column of the FK rotation, equal to `v_3`).
    fn direction(&self, theta_a: f64, theta_b: f64) -> PyResult<[f64; 3]> {
        let pose = forward_kinematics(&self.geometry, theta_a, theta_b)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        Ok([pose.v_3.x, pose.v_3.y, pose.v_3.z])
    }

    /// Whether `(θ_A, θ_B)` lands inside the workspace (loop closes).
    fn is_reachable_joints(&self, theta_a: f64, theta_b: f64) -> bool {
        forward_kinematics(&self.geometry, theta_a, theta_b).is_ok()
    }

    /// Whether `r_target` is on the reachable manifold (IK succeeds).
    fn is_reachable_orientation(&self, r_target: [[f64; 3]; 3]) -> bool {
        let r = array_to_mat3(&r_target);
        inverse_kinematics(&self.geometry, &r).is_ok()
    }

    /// Whether the pointing direction `v3` (3-vector, auto-normalized)
    /// is on the reachable manifold.
    fn is_reachable_direction(&self, v3: [f64; 3]) -> bool {
        let v = Vector3::new(v3[0], v3[1], v3[2]);
        inverse_kinematics_from_direction(&self.geometry, &v, None).is_ok()
    }

    fn arm_arc(&self) -> f64 {
        self.geometry.arm_arc
    }

    fn coupler_arc(&self) -> f64 {
        self.geometry.coupler_arc
    }

    fn mirror(&self) -> bool {
        self.geometry.mirror
    }
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

    fn close_3x3(a: &[[f64; 3]; 3], b: &Matrix3<f64>, eps: f64) -> bool {
        for i in 0..3 {
            for j in 0..3 {
                if (a[i][j] - b[(i, j)]).abs() > eps {
                    return false;
                }
            }
        }
        true
    }

    #[test]
    fn fk_default_left_at_zero_is_consistent() {
        let arm = DreamboArmKinematics::default_left();
        let m = arm.fk(0.0, 0.0).expect("FK should close at (0, 0)");
        let r = array_to_mat3(&m);

        // Columns must be orthonormal.
        let x = r.column(0);
        let y = r.column(1);
        let z = r.column(2);
        assert!((x.norm() - 1.0).abs() < 1e-9, "x not unit: {}", x.norm());
        assert!((y.norm() - 1.0).abs() < 1e-9, "y not unit: {}", y.norm());
        assert!((z.norm() - 1.0).abs() < 1e-9, "z not unit: {}", z.norm());
        assert!(x.dot(&y).abs() < 1e-9);
        assert!(y.dot(&z).abs() < 1e-9);
        assert!(x.dot(&z).abs() < 1e-9);
        // Right-handed
        assert!((x.cross(&y) - z).norm() < 1e-9);
    }

    #[test]
    fn ik_round_trip_left() {
        let arm = DreamboArmKinematics::default_left();
        let samples = [
            (0.0_f64, 0.0_f64),
            (0.2, -0.1),
            (-0.15, 0.25),
            (0.3, 0.3),
            (-0.25, -0.2),
        ];
        for &(ta, tb) in &samples {
            let m = arm.fk(ta, tb).unwrap();
            let (ta2, tb2) = arm.ik(m).unwrap();
            assert!(
                (ta - ta2).abs() < 1e-7,
                "theta_a mismatch: in={} out={}",
                ta,
                ta2
            );
            assert!(
                (tb - tb2).abs() < 1e-7,
                "theta_b mismatch: in={} out={}",
                tb,
                tb2
            );
        }
    }

    #[test]
    fn ik_round_trip_right_mirrored() {
        let arm = DreamboArmKinematics::default_right();
        let samples = [(0.0_f64, 0.0_f64), (0.2, -0.1), (-0.15, 0.2)];
        for &(ta, tb) in &samples {
            let m = arm.fk(ta, tb).unwrap();
            let (ta2, tb2) = arm.ik(m).unwrap();
            assert!((ta - ta2).abs() < 1e-7, "right theta_a: {} vs {}", ta, ta2);
            assert!((tb - tb2).abs() < 1e-7, "right theta_b: {} vs {}", tb, tb2);
        }
    }

    #[test]
    fn ik_from_direction_round_trip_left() {
        let arm = DreamboArmKinematics::default_left();
        let samples = [
            (0.0_f64, 0.0_f64),
            (0.2, -0.1),
            (-0.15, 0.25),
            (0.3, 0.3),
            (-0.25, -0.2),
        ];
        for &(ta, tb) in &samples {
            let v3 = arm.direction(ta, tb).unwrap();
            let (ta2, tb2) = arm.ik_from_direction(v3, None).unwrap();
            // Round-trip through the FK direction must reproduce v_3
            // exactly; the angles themselves may differ when the chosen
            // branch differs from `near`-free defaults, so check the
            // direction round-trip on the angles we got back.
            let v3_back = arm.direction(ta2, tb2).unwrap();
            for i in 0..3 {
                assert!(
                    (v3[i] - v3_back[i]).abs() < 1e-7,
                    "direction round-trip mismatch from ({ta},{tb}) -> ({ta2},{tb2})"
                );
            }
        }
    }

    #[test]
    fn ik_from_direction_unnormalized_input_ok() {
        let arm = DreamboArmKinematics::default_left();
        let v3 = arm.direction(0.1, -0.05).unwrap();
        let scaled = [v3[0] * 7.3, v3[1] * 7.3, v3[2] * 7.3];
        let (ta, tb) = arm.ik_from_direction(scaled, None).unwrap();
        let v3_back = arm.direction(ta, tb).unwrap();
        for i in 0..3 {
            assert!((v3[i] - v3_back[i]).abs() < 1e-7);
        }
    }

    #[test]
    fn ik_from_direction_near_picks_closest_branch() {
        let arm = DreamboArmKinematics::default_left();
        let (ta_ref, tb_ref) = (0.2_f64, -0.1_f64);
        let v3 = arm.direction(ta_ref, tb_ref).unwrap();
        let (ta, tb) = arm
            .ik_from_direction(v3, Some((ta_ref, tb_ref)))
            .unwrap();
        assert!((ta - ta_ref).abs() < 1e-7);
        assert!((tb - tb_ref).abs() < 1e-7);
    }

    #[test]
    fn ik_from_direction_zero_vector_raises() {
        let arm = DreamboArmKinematics::default_left();
        assert!(arm.ik_from_direction([0.0, 0.0, 0.0], None).is_err());
    }

    #[test]
    fn ik_from_direction_unreachable_raises() {
        let arm = DreamboArmKinematics::default_left();
        // Pointing straight down — outside the upper-hemisphere workspace
        // for the default arm geometry.
        assert!(arm.ik_from_direction([0.0, 0.0, -1.0], None).is_err());
    }

    #[test]
    fn direction_matches_third_column_of_fk() {
        let arm = DreamboArmKinematics::default_left();
        let m = arm.fk(0.15, -0.1).unwrap();
        let d = arm.direction(0.15, -0.1).unwrap();
        for i in 0..3 {
            assert!((d[i] - m[i][2]).abs() < 1e-12);
        }
    }

    #[test]
    fn unreachable_orientation_raises() {
        let arm = DreamboArmKinematics::default_left();
        // A rotation that points z_EE far from the reachable cone.
        let r_bad = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0,
        );
        let result = inverse_kinematics(&arm.geometry, &r_bad);
        assert!(result.is_err(), "Expected Unreachable for far-away target");
    }

    #[test]
    fn from_json_loads_geometry() {
        let json = r#"{
            "arm_arc": 0.7,
            "coupler_arc": 0.5,
            "branch": "positive",
            "mirror": false
        }"#;
        let arm = DreamboArmKinematics::from_json(json).unwrap();
        assert!((arm.arm_arc() - 0.7).abs() < 1e-12);
        assert!((arm.coupler_arc() - 0.5).abs() < 1e-12);
        assert!(!arm.mirror());

        // Verify a round-trip on the parsed geometry.
        let m = arm.fk(0.1, 0.1).unwrap();
        let (a, b) = arm.ik(m).unwrap();
        assert!((a - 0.1).abs() < 1e-7);
        assert!((b - 0.1).abs() < 1e-7);
    }

    #[test]
    fn _check_close_3x3_used() {
        // Compile guard so close_3x3 isn't dead code if tests change.
        let i = Matrix3::<f64>::identity();
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        assert!(close_3x3(&a, &i, 1e-12));
    }
}
