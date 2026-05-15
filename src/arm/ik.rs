//! Inverse kinematics for the spherical 5-bar arm.
//!
//! Strategy: the two small-circle intersections that locate `v_2` and
//! `v_4` each have up to two solutions. We enumerate the four
//! combinations, compute candidate `(θ_A, θ_B)` for each, and pick the
//! one whose forward kinematics reproduces the target rotation.

use nalgebra::{Matrix3, Unit, Vector3};

use super::fk::{forward_kinematics, intersect_small_circles};
use super::geometry::SphericalLinkGeometry;

#[derive(Debug, thiserror::Error)]
pub enum IkError {
    /// The desired orientation is outside the reachable workspace.
    #[error("Target orientation is outside the spherical 5-bar workspace.")]
    Unreachable,
}

const VERIFY_EPS: f64 = 1e-6;

/// Recover `(θ_A, θ_B)` from a target end-effector rotation.
pub fn inverse_kinematics(
    geom: &SphericalLinkGeometry,
    r_target: &Matrix3<f64>,
) -> Result<(f64, f64), IkError> {
    let v_3 = r_target.column(2).into_owned();
    let z_a = geom.z_a();
    let z_b = geom.z_b();
    let arm = geom.arm_arc;
    let coup = geom.coupler_arc;
    let v_2_ref = geom.v_2_ref();
    let v_4_ref = geom.v_4_ref();

    let mut best: Option<((f64, f64), f64)> = None;

    for &s_a in &[1.0_f64, -1.0_f64] {
        let v_2 = match intersect_small_circles(&z_a, arm, &v_3, coup, s_a) {
            Some(v) => v,
            None => continue,
        };
        let theta_a = rotation_angle_about(&z_a, &v_2_ref, &v_2);

        for &s_b in &[1.0_f64, -1.0_f64] {
            let v_4 = match intersect_small_circles(&z_b, arm, &v_3, coup, s_b) {
                Some(v) => v,
                None => continue,
            };
            let theta_b = rotation_angle_about(&z_b, &v_4_ref, &v_4);

            // Verify via FK and pick the best-matching candidate.
            let pose = match forward_kinematics(geom, theta_a, theta_b) {
                Ok(p) => p,
                Err(_) => continue,
            };
            let err = (pose.r_world_ee - r_target).norm();
            if err < VERIFY_EPS && best.as_ref().map_or(true, |(_, e)| err < *e) {
                best = Some(((theta_a, theta_b), err));
            }
        }
    }

    best.map(|(joints, _)| joints).ok_or(IkError::Unreachable)
}

/// Signed rotation angle about `axis` that takes `v_ref` to `v_target`.
fn rotation_angle_about(
    axis: &Vector3<f64>,
    v_ref: &Vector3<f64>,
    v_target: &Vector3<f64>,
) -> f64 {
    let axis_u = Unit::new_normalize(*axis);
    let axis_v = axis_u.into_inner();
    let proj_ref = *v_ref - axis_v * axis_v.dot(v_ref);
    let proj_target = *v_target - axis_v * axis_v.dot(v_target);
    let cross = proj_ref.cross(&proj_target);
    let sin_part = axis_v.dot(&cross);
    let cos_part = proj_ref.dot(&proj_target);
    sin_part.atan2(cos_part)
}
