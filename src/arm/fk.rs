//! Forward kinematics for the spherical 5-bar arm.

use nalgebra::{Matrix3, Rotation3, Unit, Vector3};

use super::geometry::SphericalLinkGeometry;

/// Result of [`forward_kinematics`].
#[derive(Debug, Clone, Copy)]
pub struct ArmPose {
    /// 3×3 rotation taking the end-effector frame into the body frame.
    pub r_world_ee: Matrix3<f64>,
    /// Position of `v_2` (arm A passive joint) on the unit sphere.
    pub v_2: Vector3<f64>,
    /// Position of `v_3` (middle passive joint) on the unit sphere.
    pub v_3: Vector3<f64>,
    /// Position of `v_4` (arm B passive joint) on the unit sphere.
    pub v_4: Vector3<f64>,
}

#[derive(Debug, thiserror::Error)]
pub enum FkError {
    /// The two small circles defining `v_3` do not intersect; `v_2` and
    /// `v_4` are more than `2 · coupler_arc` apart on the sphere.
    #[error("Spherical 5-bar loop does not close: v_2 and v_4 are too far apart for the coupler.")]
    LoopOpen,
}

/// Compute `v_2`, `v_3`, `v_4` and the end-effector rotation from joint
/// angles `(θ_A, θ_B)`.
pub fn forward_kinematics(
    geom: &SphericalLinkGeometry,
    theta_a: f64,
    theta_b: f64,
) -> Result<ArmPose, FkError> {
    let z_a = Unit::new_normalize(geom.z_a());
    let z_b = Unit::new_normalize(geom.z_b());

    let r_a = Rotation3::from_axis_angle(&z_a, theta_a);
    let r_b = Rotation3::from_axis_angle(&z_b, theta_b);

    let v_2 = r_a * geom.v_2_ref();
    let v_4 = r_b * geom.v_4_ref();

    let v_3 = intersect_small_circles(&v_2, geom.coupler_arc, &v_4, geom.coupler_arc, geom.branch.sign())
        .ok_or(FkError::LoopOpen)?;

    let r_world_ee = build_ee_frame(&v_2, &v_3, &v_4);

    Ok(ArmPose {
        r_world_ee,
        v_2,
        v_3,
        v_4,
    })
}

/// Find a unit vector `p` on the sphere with `p · a = cos r_a` and
/// `p · b = cos r_b`. Two solutions in general; `branch_sign` (±1) picks
/// one. Returns `None` if the two small circles do not intersect.
pub fn intersect_small_circles(
    a: &Vector3<f64>,
    r_a: f64,
    b: &Vector3<f64>,
    r_b: f64,
    branch_sign: f64,
) -> Option<Vector3<f64>> {
    let cos_ab = a.dot(b).clamp(-1.0, 1.0);
    let sin_ab = (1.0 - cos_ab * cos_ab).sqrt();
    if sin_ab < 1e-12 {
        // a and b are (anti-)parallel — small circles either coincide
        // (infinite solutions) or are disjoint. Treat as no closure.
        return None;
    }

    let e1 = *a;
    let e3 = a.cross(b) / sin_ab;
    let e2 = e3.cross(&e1);

    let cos_ra = r_a.cos();
    let cos_rb = r_b.cos();
    let px = cos_ra;
    let py = (cos_rb - cos_ra * cos_ab) / sin_ab;
    let pz_sq = 1.0 - px * px - py * py;
    if pz_sq < 0.0 {
        return None;
    }
    let pz = branch_sign * pz_sq.sqrt();

    Some(px * e1 + py * e2 + pz * e3)
}

/// Build the end-effector rotation from `(v_2, v_3, v_4)`.
///
/// `z_EE = v_3`, `x_EE` is the geodesic bisector of `v_2` and `v_4`
/// projected onto the plane perpendicular to `v_3`, and `y_EE` closes
/// the right-handed frame.
fn build_ee_frame(v_2: &Vector3<f64>, v_3: &Vector3<f64>, v_4: &Vector3<f64>) -> Matrix3<f64> {
    let bisector = (v_2 + v_4).normalize();
    let z = *v_3;
    // Project bisector onto plane perpendicular to z.
    let x_unnorm = bisector - z * z.dot(&bisector);
    let x_norm = x_unnorm.norm();
    let x = if x_norm > 1e-12 {
        x_unnorm / x_norm
    } else {
        // Degenerate: bisector is parallel to z_EE. Fall back to a stable
        // orthonormal frame perpendicular to z.
        any_perpendicular(&z)
    };
    let y = z.cross(&x);

    Matrix3::from_columns(&[x, y, z])
}

fn any_perpendicular(v: &Vector3<f64>) -> Vector3<f64> {
    let candidate = if v.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let perp = candidate - v * v.dot(&candidate);
    perp.normalize()
}
