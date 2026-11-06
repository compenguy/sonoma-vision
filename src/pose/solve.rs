//! Camera pose from points: IPPE (Collins and Bartoli, "Infinitesimal Plane-Based Pose Estimation",
//! IJCV 2014) for a planar target's two candidate poses, and Levenberg-Marquardt refinement of the
//! reprojection error over any set of points. Poses here take points into the camera's optical frame
//! (x right, y down, z forward); observations are normalised image coordinates (see
//! [`super::camera::Intrinsics::normalize`]).

use nalgebra::{
    Isometry3, Matrix2, Matrix3, Matrix6, Rotation3, SMatrix, SVector, Translation3, UnitQuaternion, Vector3,
};

/// The homography taking plane points `(x, y)` (z = 0) to normalised image points, from exactly four
/// correspondences. None if they're degenerate (three in a line).
fn homography(plane: &[[f64; 2]; 4], image: &[[f64; 2]; 4]) -> Option<Matrix3<f64>> {
    // h00 x + h01 y + h02 - u (h20 x + h21 y) = u, and likewise for v, with h22 = 1.
    let mut a = SMatrix::<f64, 8, 8>::zeros();
    let mut b = SVector::<f64, 8>::zeros();
    for (i, (&[x, y], &[u, v])) in plane.iter().zip(image).enumerate() {
        let (r0, r1) = (2 * i, 2 * i + 1);
        a.row_mut(r0).copy_from_slice(&[x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y]);
        a.row_mut(r1).copy_from_slice(&[0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y]);
        b[r0] = u;
        b[r1] = v;
    }
    let h = a.lu().solve(&b)?;
    let m = Matrix3::new(h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0);
    m.iter().all(|v| v.is_finite()).then_some(m)
}

/// IPPE's two rotations (plane frame to camera) for the plane whose homography to normalised image
/// coordinates is `h`. They are the two poses the plane's first-order appearance at its origin
/// allows: one is right, and the other is its mirror image about the line of sight.
fn ippe_rotations(h: &Matrix3<f64>) -> Option<[Matrix3<f64>; 2]> {
    let h = h / h[(2, 2)];
    // Where the plane's origin appears, and the homography's Jacobian there.
    let (p, q) = (h[(0, 2)], h[(1, 2)]);
    let j = Matrix2::new(
        h[(0, 0)] - h[(2, 0)] * p,
        h[(0, 1)] - h[(2, 1)] * p,
        h[(1, 0)] - h[(2, 0)] * q,
        h[(1, 1)] - h[(2, 1)] * q,
    );
    // Rv turns the optical axis onto the ray through the origin's image.
    let rv = Rotation3::rotation_between(&Vector3::z(), &Vector3::new(p, q, 1.0))?.into_inner();
    let b = Matrix2::new(
        rv[(0, 0)] - p * rv[(2, 0)],
        rv[(0, 1)] - p * rv[(2, 1)],
        rv[(1, 0)] - q * rv[(2, 0)],
        rv[(1, 1)] - q * rv[(2, 1)],
    );
    let a = b.try_inverse()? * j;
    // The largest singular value of A scales it to the top-left of a rotation matrix.
    let ata = a * a.transpose();
    let (s00, s01, s11) = (ata[(0, 0)], ata[(0, 1)], ata[(1, 1)]);
    let gamma = (0.5 * (s00 + s11 + ((s00 - s11).powi(2) + 4.0 * s01 * s01).sqrt())).sqrt();
    if gamma.is_nan() || gamma <= 0.0 {
        return None;
    }
    let r = a / gamma;
    let b0 = (1.0 - r[(0, 0)].powi(2) - r[(1, 0)].powi(2)).max(0.0).sqrt();
    let mut b1 = (1.0 - r[(0, 1)].powi(2) - r[(1, 1)].powi(2)).max(0.0).sqrt();
    if -r[(0, 0)] * r[(0, 1)] - r[(1, 0)] * r[(1, 1)] < 0.0 {
        b1 = -b1;
    }
    Some([1.0, -1.0].map(|s| {
        let c0 = Vector3::new(r[(0, 0)], r[(1, 0)], s * b0);
        let c1 = Vector3::new(r[(0, 1)], r[(1, 1)], s * b1);
        rv * Matrix3::from_columns(&[c0, c1, c0.cross(&c1)])
    }))
}

/// The translation that best places `world` points, rotated by `r`, onto the observations (linear
/// least squares).
fn translation_for(r: &Matrix3<f64>, world: &[Vector3<f64>], obs: &[[f64; 2]]) -> Option<Vector3<f64>> {
    // t_x - u t_z = u (R X)_z - (R X)_x, and likewise for v.
    let (mut ata, mut atb) = (Matrix3::zeros(), Vector3::zeros());
    for (x, &[u, v]) in world.iter().zip(obs) {
        let rx = r * x;
        for (row, rhs) in [(Vector3::new(1.0, 0.0, -u), u * rx.z - rx.x), (Vector3::new(0.0, 1.0, -v), v * rx.z - rx.y)]
        {
            ata += row * row.transpose();
            atb += row * rhs;
        }
    }
    ata.try_inverse().map(|m| m * atb)
}

/// Sum of squared reprojection errors (normalised coordinates), or None if a point is behind the camera.
fn cost(pose: &Isometry3<f64>, world: &[Vector3<f64>], obs: &[[f64; 2]]) -> Option<f64> {
    let mut sum = 0.0;
    for (x, &[u, v]) in world.iter().zip(obs) {
        let c = pose * nalgebra::Point3::from(*x);
        if c.z <= 1e-9 {
            return None;
        }
        sum += (c.x / c.z - u).powi(2) + (c.y / c.z - v).powi(2);
    }
    Some(sum)
}

/// Refine `pose` (world to optical camera) to minimise the reprojection error of `world` points
/// against the observations: Levenberg-Marquardt over a rotation increment and the translation.
/// Returns the final cost, or None if the starting pose puts a point behind the camera.
pub fn refine(pose: &mut Isometry3<f64>, world: &[Vector3<f64>], obs: &[[f64; 2]]) -> Option<f64> {
    let mut current = cost(pose, world, obs)?;
    let mut lambda = 1e-3;
    for _ in 0..50 {
        let (mut jtj, mut jte) = (Matrix6::<f64>::zeros(), SVector::<f64, 6>::zeros());
        for (x, &[u, v]) in world.iter().zip(obs) {
            let rx = pose.rotation * x;
            let c = rx + pose.translation.vector;
            let iz = 1.0 / c.z;
            // d(projection)/d(camera point), and d(camera point)/d(rotation increment, translation).
            let dp = SMatrix::<f64, 2, 3>::new(iz, 0.0, -c.x * iz * iz, 0.0, iz, -c.y * iz * iz);
            let mut dc = SMatrix::<f64, 3, 6>::zeros();
            dc.fixed_view_mut::<3, 3>(0, 0).copy_from(&-rx.cross_matrix());
            dc.fixed_view_mut::<3, 3>(0, 3).copy_from(&Matrix3::identity());
            let jac = dp * dc;
            let e = nalgebra::Vector2::new(c.x * iz - u, c.y * iz - v);
            jtj += jac.transpose() * jac;
            jte += jac.transpose() * e;
        }
        let mut improved = false;
        while lambda < 1e8 {
            let mut damped = jtj;
            for i in 0..6 {
                damped[(i, i)] += lambda * jtj[(i, i)].max(1e-12);
            }
            let Some(step) = damped.cholesky().map(|c| -c.solve(&jte)) else {
                lambda *= 10.0;
                continue;
            };
            let rotation = UnitQuaternion::from_scaled_axis(step.fixed_rows::<3>(0).into_owned()) * pose.rotation;
            let translation = Translation3::from(pose.translation.vector + step.fixed_rows::<3>(3));
            let candidate = Isometry3::from_parts(translation, rotation);
            match cost(&candidate, world, obs) {
                Some(c) if c < current => {
                    let gain = current - c;
                    *pose = candidate;
                    current = c;
                    lambda = (lambda / 10.0).max(1e-9);
                    improved = gain > 1e-14 * (1.0 + c);
                    break;
                }
                _ => lambda *= 10.0,
            }
        }
        if !improved {
            break;
        }
    }
    Some(current)
}

/// Both poses of a square (or any planar four-point) target whose plane points (z = 0) are `plane`,
/// seen at the normalised `image` points: IPPE's two candidates, each refined. Lowest cost first.
pub fn solve_planar(plane: &[[f64; 2]; 4], image: &[[f64; 2]; 4]) -> Vec<(Isometry3<f64>, f64)> {
    let Some(h) = homography(plane, image) else { return Vec::new() };
    let Some(rotations) = ippe_rotations(&h) else { return Vec::new() };
    let world = plane.map(|[x, y]| Vector3::new(x, y, 0.0));
    let mut out: Vec<(Isometry3<f64>, f64)> = rotations
        .iter()
        .filter_map(|r| {
            let t = translation_for(r, &world, image)?;
            let rotation = UnitQuaternion::from_matrix(r);
            let mut pose = Isometry3::from_parts(Translation3::from(t), rotation);
            let cost = refine(&mut pose, &world, image)?;
            Some((pose, cost))
        })
        .collect();
    out.sort_by(|a, b| a.1.total_cmp(&b.1));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn project(pose: &Isometry3<f64>, p: &Vector3<f64>) -> [f64; 2] {
        let c = pose * nalgebra::Point3::from(*p);
        [c.x / c.z, c.y / c.z]
    }

    const SQUARE: [[f64; 2]; 4] = [[-0.08, -0.08], [0.08, -0.08], [0.08, 0.08], [-0.08, 0.08]];

    #[test]
    fn recovers_exact_poses() {
        for (axis, angle, t) in [
            (Vector3::y(), 0.0, Vector3::new(0.0, 0.0, 2.0)),
            (Vector3::y(), 0.7, Vector3::new(0.3, -0.2, 1.5)),
            (Vector3::new(1.0, 1.0, 0.2), 1.0, Vector3::new(-0.5, 0.4, 3.0)),
            (Vector3::x(), -1.2, Vector3::new(0.1, 0.0, 0.8)),
        ] {
            let truth = Isometry3::from_parts(
                Translation3::from(t),
                UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle),
            );
            let image = SQUARE.map(|[x, y]| project(&truth, &Vector3::new(x, y, 0.0)));
            let solutions = solve_planar(&SQUARE, &image);
            let (best, cost) = solutions[0];
            assert!(cost < 1e-20, "cost {cost}");
            assert!((best.translation.vector - t).norm() < 1e-9, "{:?} vs {t:?}", best.translation);
            assert!(best.rotation.angle_to(&truth.rotation) < 1e-9);
        }
    }

    #[test]
    fn refines_many_points() {
        let truth =
            Isometry3::from_parts(Translation3::new(0.2, -0.1, 4.0), UnitQuaternion::from_euler_angles(0.1, -0.3, 0.2));
        let world: Vec<Vector3<f64>> = (0..12)
            .map(|i| Vector3::new((i % 4) as f64 * 0.5 - 0.7, (i / 4) as f64 * 0.4, (i % 3) as f64 * 0.3))
            .collect();
        let obs: Vec<[f64; 2]> = world.iter().map(|p| project(&truth, p)).collect();
        // Start well off.
        let mut pose =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 3.0), UnitQuaternion::from_euler_angles(0.0, -0.1, 0.0));
        let cost = refine(&mut pose, &world, &obs).unwrap();
        assert!(cost < 1e-20, "cost {cost}");
        assert!((pose.translation.vector - truth.translation.vector).norm() < 1e-9);
    }
}
