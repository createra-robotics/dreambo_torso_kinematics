//! Inverse kinematics for the spherical 5-bar arm.
//!
//! Strategy: the two small-circle intersections that locate `v_2` and
//! `v_4` each have up to two solutions. We enumerate the four
//! combinations, compute candidate `(θ_A, θ_B)` for each, then pick the
//! winning candidate using a ranking that depends on what the caller
//! gave us — a full target rotation, or just the pointing direction.

use nalgebra::{Matrix3, Unit, Vector3};

use super::fk::{forward_kinematics, intersect_small_circles, ArmPose};
use super::geometry::SphericalLinkGeometry;

#[derive(Debug, thiserror::Error)]
pub enum IkError {
    /// The desired orientation is outside the reachable workspace.
    #[error("Target orientation is outside the spherical 5-bar workspace.")]
    Unreachable,
}

const VERIFY_EPS: f64 = 1e-6;
const DIRECTION_EPS: f64 = 1e-6;

/// Enumerate every `(θ_A, θ_B, ArmPose)` candidate produced by the four
/// `(s_a, s_b)` small-circle branches for a given pointing direction.
fn enumerate_candidates(
    geom: &SphericalLinkGeometry,
    v_3: &Vector3<f64>,
) -> Vec<(f64, f64, ArmPose)> {
    let z_a = geom.z_a();
    let z_b = geom.z_b();
    let arm = geom.arm_arc;
    let coup = geom.coupler_arc;
    let v_2_ref = geom.v_2_ref();
    let v_4_ref = geom.v_4_ref();

    let mut out = Vec::with_capacity(4);
    for &s_a in &[1.0_f64, -1.0_f64] {
        let v_2 = match intersect_small_circles(&z_a, arm, v_3, coup, s_a) {
            Some(v) => v,
            None => continue,
        };
        let theta_a = rotation_angle_about(&z_a, &v_2_ref, &v_2);

        for &s_b in &[1.0_f64, -1.0_f64] {
            let v_4 = match intersect_small_circles(&z_b, arm, v_3, coup, s_b) {
                Some(v) => v,
                None => continue,
            };
            let theta_b = rotation_angle_about(&z_b, &v_4_ref, &v_4);

            let pose = match forward_kinematics(geom, theta_a, theta_b) {
                Ok(p) => p,
                Err(_) => continue,
            };
            out.push((theta_a, theta_b, pose));
        }
    }
    out
}

/// Recover `(θ_A, θ_B)` from a target end-effector rotation.
pub fn inverse_kinematics(
    geom: &SphericalLinkGeometry,
    r_target: &Matrix3<f64>,
) -> Result<(f64, f64), IkError> {
    let v_3 = r_target.column(2).into_owned();
    let mut best: Option<((f64, f64), f64)> = None;
    for (theta_a, theta_b, pose) in enumerate_candidates(geom, &v_3) {
        let err = (pose.r_world_ee - r_target).norm();
        if err < VERIFY_EPS && best.as_ref().map_or(true, |(_, e)| err < *e) {
            best = Some(((theta_a, theta_b), err));
        }
    }
    best.map(|(joints, _)| joints).ok_or(IkError::Unreachable)
}

/// Recover `(θ_A, θ_B)` from just the end-effector pointing direction
/// (`v_3` on the unit sphere). Multiple branches can reach the same
/// direction; `near`, when supplied, picks the candidate closest to a
/// reference joint pair, otherwise the smallest-norm candidate wins.
///
/// The input vector is normalized internally; passing the zero vector
/// raises `Unreachable`.
pub fn inverse_kinematics_from_direction(
    geom: &SphericalLinkGeometry,
    v_3_target: &Vector3<f64>,
    near: Option<(f64, f64)>,
) -> Result<(f64, f64), IkError> {
    let norm = v_3_target.norm();
    if norm < 1e-12 {
        return Err(IkError::Unreachable);
    }
    let v_3 = v_3_target / norm;

    let candidates: Vec<(f64, f64)> = enumerate_candidates(geom, &v_3)
        .into_iter()
        .filter(|(_, _, pose)| (pose.v_3 - v_3).norm() < DIRECTION_EPS)
        .map(|(theta_a, theta_b, _)| (theta_a, theta_b))
        .collect();

    let cost = |t: &(f64, f64)| -> f64 {
        match near {
            Some((ta, tb)) => (t.0 - ta).powi(2) + (t.1 - tb).powi(2),
            None => t.0.powi(2) + t.1.powi(2),
        }
    };

    candidates
        .into_iter()
        .min_by(|a, b| {
            cost(a)
                .partial_cmp(&cost(b))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .ok_or(IkError::Unreachable)
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