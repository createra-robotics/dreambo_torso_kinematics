//! Static geometry of the spherical 5-bar arm.
//!
//! Conventions:
//! - All five revolute axes intersect at the sphere center (origin).
//! - Ground axes are orthogonal: `z_A = (0, 0, 1)` (the yaw motor),
//!   `z_B = (1, 0, 0)` (the pitch motor). For the mirrored ("right")
//!   arm, `z_B` becomes `(-1, 0, 0)`.
//! - At `θ_A = 0` the input arm A is in the XZ plane, swung from `z_A`
//!   toward `z_B`. At `θ_B = 0` arm B is similarly in the XZ plane.
//! - The two coupler-side passive joints (`v_2` for arm A, `v_4` for
//!   arm B) live at arc length `arm_arc` from their ground axes. The
//!   middle passive joint `v_3` lives at arc length `coupler_arc` from
//!   both `v_2` and `v_4`.
//! - The end-effector frame has `z_EE = v_3`; `x_EE` is the geodesic
//!   bisector of `v_2` and `v_4`, projected perpendicular to `v_3`;
//!   `y_EE = z_EE × x_EE`.
//! - A `branch` flag picks the assembly mode for the small-circle
//!   intersection that determines `v_3`.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Branch {
    /// Pick the `v_3` solution with positive cross-product sign.
    Positive,
    /// Pick the `v_3` solution with negative cross-product sign.
    Negative,
}

impl Branch {
    pub fn sign(&self) -> f64 {
        match self {
            Branch::Positive => 1.0,
            Branch::Negative => -1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SphericalLinkGeometry {
    /// Arc length of either input arm (radians). Symmetric: arm A = arm B.
    pub arm_arc: f64,
    /// Arc length of either coupler segment (radians). Symmetric: from
    /// `v_2` to `v_3` and from `v_4` to `v_3` are equal.
    pub coupler_arc: f64,
    /// Assembly mode for the small-circle intersection.
    #[serde(default = "default_branch")]
    pub branch: Branch,
    /// When true, mirror the arm across the YZ plane (swap left ↔ right).
    /// Concretely: `z_B = (-1, 0, 0)` and the reference for both `v_2`
    /// and `v_4` is reflected in X.
    #[serde(default)]
    pub mirror: bool,
}

fn default_branch() -> Branch {
    Branch::Positive
}

impl SphericalLinkGeometry {
    /// Default left-arm geometry. Placeholder values pending CAD numbers.
    pub fn default_left() -> Self {
        Self {
            arm_arc: 0.7,
            coupler_arc: 0.5,
            branch: Branch::Positive,
            mirror: false,
        }
    }

    /// Default right-arm geometry: same arcs, mirrored across YZ.
    pub fn default_right() -> Self {
        Self {
            mirror: true,
            ..Self::default_left()
        }
    }

    /// Ground axis A (yaw motor). Always vertical.
    pub fn z_a(&self) -> Vector3<f64> {
        Vector3::new(0.0, 0.0, 1.0)
    }

    /// Ground axis B (pitch motor). Flipped for the mirrored arm.
    pub fn z_b(&self) -> Vector3<f64> {
        if self.mirror {
            Vector3::new(-1.0, 0.0, 0.0)
        } else {
            Vector3::new(1.0, 0.0, 0.0)
        }
    }

    /// Reference position of `v_2` (joint between arm A and coupler) at θ_A = 0.
    /// Arc `arm_arc` from `z_A` toward `z_B`, in the XZ plane.
    pub fn v_2_ref(&self) -> Vector3<f64> {
        let s = self.arm_arc.sin();
        let c = self.arm_arc.cos();
        let x = if self.mirror { -s } else { s };
        Vector3::new(x, 0.0, c)
    }

    /// Reference position of `v_4` (joint between coupler and arm B) at θ_B = 0.
    /// Arc `arm_arc` from `z_B` toward `z_A`, in the XZ plane.
    pub fn v_4_ref(&self) -> Vector3<f64> {
        let s = self.arm_arc.sin();
        let c = self.arm_arc.cos();
        let x = if self.mirror { -c } else { c };
        Vector3::new(x, 0.0, s)
    }
}
