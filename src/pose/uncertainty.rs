//! Pose uncertainty: a pose's covariance from how well its corners pin it down, carried between
//! frames, mixed over a pose's rival solutions, and reduced to what WPILib's pose estimators take.
//!
//! A covariance here is 6×6 over `[x, y, z, rx, ry, rz]`: the error in the pose's translation, then a
//! small rotation `r` with the true rotation `exp([r]×)·R`, both along the parent frame's axes (the
//! field's, for a pose on the field). The methods are the textbook ones: the first-order (Gauss-Newton)
//! covariance of a least-squares fit, first-order propagation through rigid transforms, and moment
//! matching over a mixture of solutions.

use nalgebra::{Isometry3, Matrix3, Matrix6, Vector3, Vector6};

/// `pose`'s offset to `other`: `[t_other - t, r]` with `R_other = exp([r]×)·R`.
pub fn difference(pose: &Isometry3<f64>, other: &Isometry3<f64>) -> Vector6<f64> {
    let r = (other.rotation * pose.rotation.inverse()).scaled_axis();
    let t = other.translation.vector - pose.translation.vector;
    Vector6::new(t.x, t.y, t.z, r.x, r.y, r.z)
}

/// The covariance of the camera's pose in the world (its optical frame's, the inverse of
/// `world_to_optical`), from the covariance of `world_to_optical`'s increment as
/// [`super::solve::information`] has it: `[r, t]`, the rotation applied on the left.
pub fn camera_in_world(world_to_optical: &Isometry3<f64>, increment: &Matrix6<f64>) -> Matrix6<f64> {
    // With x_o = R x_w + t, the camera is at p = -Rᵀt with rotation Rᵀ. Turning R by exp([δr]×) and
    // moving t by δt moves p by -Rᵀ[t]×δr - Rᵀδt, and turns the camera by -Rᵀδr in world axes.
    let rt = world_to_optical.rotation.to_rotation_matrix().into_inner().transpose();
    let t = world_to_optical.translation.vector;
    let mut g = Matrix6::zeros();
    g.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-rt * t.cross_matrix()));
    g.fixed_view_mut::<3, 3>(0, 3).copy_from(&-rt);
    g.fixed_view_mut::<3, 3>(3, 0).copy_from(&-rt);
    g * increment * g.transpose()
}

/// The covariance of `pose * offset`, a frame rigidly attached to `pose`'s (the robot's to the
/// camera's), from `pose`'s. `offset` is taken as exact.
pub fn attached(pose: &Isometry3<f64>, offset: &Isometry3<f64>, cov: &Matrix6<f64>) -> Matrix6<f64> {
    // The attached frame's origin is a = R·offset.t from pose's, so a small rotation r moves it by r × a.
    let a = pose.rotation * offset.translation.vector;
    let mut h = Matrix6::identity();
    h.fixed_view_mut::<3, 3>(0, 3).copy_from(&-a.cross_matrix());
    h * cov * h.transpose()
}

/// The second moment about `pose` of a mixture of solutions, each `(solution, covariance, weight)`:
/// `Σ w (P + d dᵀ)`, with `d` each solution's [`difference`] from `pose` and the weights normalised.
/// One solution at `pose` gives its own covariance; a rival elsewhere widens it to cover both, as
/// much as its weight says.
pub fn mixture(pose: &Isometry3<f64>, members: &[(Isometry3<f64>, Matrix6<f64>, f64)]) -> Matrix6<f64> {
    let total: f64 = members.iter().map(|m| m.2).sum();
    members.iter().fold(Matrix6::zeros(), |acc, (solution, cov, w)| {
        let d = difference(pose, solution);
        acc + (cov + d * d.transpose()) * (w / total)
    })
}

/// Add `fraction` of each variance back onto itself. A first-order covariance can claim that two
/// axes' errors cancel (a rotation and the sideways shift that mimics it, for a distant tag) far
/// better than they really do, when the fit is curved over the noise's reach. This caps every
/// correlation at `1 / (1 + fraction)`, and widens each standard deviation by `√(1 + fraction)`.
pub fn inflate<const N: usize>(cov: &nalgebra::SMatrix<f64, N, N>, fraction: f64) -> nalgebra::SMatrix<f64, N, N> {
    let mut out = *cov;
    for i in 0..N {
        out[(i, i)] *= 1.0 + fraction;
    }
    out
}

/// A pose projected onto the floor, as WPILib's `Pose2d`, with its covariance over `[x, y, yaw]`
/// (x and y along the parent's axes).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2d {
    pub x: f64,
    pub y: f64,
    /// The heading of the pose's +X axis, counter-clockwise from the parent's +X, in radians.
    pub yaw: f64,
    pub covariance: Matrix3<f64>,
}

impl Pose2d {
    /// `pose` and its 6×6 covariance, projected onto the parent's X/Y plane. The yaw's variance
    /// takes in roll and pitch errors as far as they turn the heading.
    pub fn project(pose: &Isometry3<f64>, cov: &Matrix6<f64>) -> Self {
        let h = pose.rotation * Vector3::x();
        let flat = (h.x * h.x + h.y * h.y).max(1e-12);
        // yaw = atan2(h_y, h_x); a small rotation r moves h by r × h.
        let mut j = nalgebra::SMatrix::<f64, 3, 6>::zeros();
        j[(0, 0)] = 1.0;
        j[(1, 1)] = 1.0;
        j[(2, 3)] = -h.x * h.z / flat;
        j[(2, 4)] = -h.y * h.z / flat;
        j[(2, 5)] = 1.0;
        let t = pose.translation.vector;
        Self { x: t.x, y: t.y, yaw: h.y.atan2(h.x), covariance: j * cov * j.transpose() }
    }

    /// The standard deviations of x and y along the pose's own forward and left axes, and of yaw:
    /// what WPILib's `addVisionMeasurement` takes, as its estimators apply them in the robot's frame.
    pub fn local_std_devs(&self) -> [f64; 3] {
        let (s, c) = self.yaw.sin_cos();
        let to_local = Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0);
        let local = to_local * self.covariance * to_local.transpose();
        [0, 1, 2].map(|i| local[(i, i)].max(0.0).sqrt())
    }

    /// As [`mixture`], for floor poses: the second moment about `self` of the weighted members.
    pub fn mixture(&self, members: &[(Pose2d, f64)]) -> Matrix3<f64> {
        let total: f64 = members.iter().map(|m| m.1).sum();
        members.iter().fold(Matrix3::zeros(), |acc, (m, w)| {
            let dyaw =
                (m.yaw - self.yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
            let d = Vector3::new(m.x - self.x, m.y - self.y, dyaw);
            acc + (m.covariance + d * d.transpose()) * (w / total)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    /// Apply a small `[t, r]` perturbation to `pose`, in the parent's axes.
    fn perturb(pose: &Isometry3<f64>, d: &Vector6<f64>) -> Isometry3<f64> {
        let r = UnitQuaternion::from_scaled_axis(Vector3::new(d[3], d[4], d[5]));
        Isometry3::from_parts(Translation3::from(pose.translation.vector + d.fixed_rows::<3>(0)), r * pose.rotation)
    }

    fn sample_pose() -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(1.0, -2.0, 0.5), UnitQuaternion::from_euler_angles(0.2, -0.3, 1.1))
    }

    #[test]
    fn camera_in_world_matches_a_numeric_derivative() {
        let w2o = sample_pose();
        let eps = 1e-6;
        // Column k of the map from the increment [r, t] to the camera's [t, r] in the world.
        for k in 0..6 {
            let mut inc = Vector6::zeros();
            inc[k] = eps;
            let rot = UnitQuaternion::from_scaled_axis(Vector3::new(inc[0], inc[1], inc[2]));
            let moved = Isometry3::from_parts(
                Translation3::from(w2o.translation.vector + Vector3::new(inc[3], inc[4], inc[5])),
                rot * w2o.rotation,
            );
            let numeric = difference(&w2o.inverse(), &moved.inverse()) / eps;
            // The analytic map, applied to a unit increment, through the covariance of e_k e_kᵀ.
            let mut unit = Matrix6::zeros();
            unit[(k, k)] = 1.0;
            let cov = camera_in_world(&w2o, &unit);
            for i in 0..6 {
                assert!((cov[(i, i)].sqrt() - numeric[i].abs()).abs() < 1e-5, "k {k} i {i}: {cov} vs {numeric}");
            }
        }
    }

    #[test]
    fn attached_matches_a_numeric_derivative() {
        let pose = sample_pose();
        let offset =
            Isometry3::from_parts(Translation3::new(-0.3, 0.1, -0.5), UnitQuaternion::from_euler_angles(0.0, 0.3, 0.0));
        let eps = 1e-6;
        for k in 0..6 {
            let mut d = Vector6::zeros();
            d[k] = eps;
            let numeric = difference(&(pose * offset), &(perturb(&pose, &d) * offset)) / eps;
            let mut unit = Matrix6::zeros();
            unit[(k, k)] = 1.0;
            let cov = attached(&pose, &offset, &unit);
            for i in 0..6 {
                assert!((cov[(i, i)].sqrt() - numeric[i].abs()).abs() < 1e-5, "k {k} i {i}");
            }
        }
    }

    #[test]
    fn local_std_devs_turn_with_the_heading() {
        // Uncertain along the field's X only; facing along the field's Y, that's sideways (left).
        let mut cov = Matrix6::zeros();
        cov[(0, 0)] = 0.04;
        let facing_y = Isometry3::from_parts(
            Translation3::new(1.0, 1.0, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );
        let [fwd, left, yaw] = Pose2d::project(&facing_y, &cov).local_std_devs();
        assert!(fwd < 1e-9 && (left - 0.2).abs() < 1e-9 && yaw < 1e-9, "{fwd} {left} {yaw}");
    }

    #[test]
    fn mixture_covers_a_rival() {
        let pose = sample_pose();
        let cov = Matrix6::identity() * 1e-4;
        let rival = perturb(&pose, &Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.2));
        assert_eq!(mixture(&pose, &[(pose, cov, 1.0)]), cov);
        let both = mixture(&pose, &[(pose, cov, 1.0), (rival, cov, 1.0)]);
        // Half the weight 0.2 rad away: rz's variance gains 0.02.
        assert!((both[(5, 5)] - (1e-4 + 0.02)).abs() < 1e-9, "{both}");
        assert!((inflate(&cov, 0.1)[(0, 0)] - 1.1e-4).abs() < 1e-12);
    }
}
