//! End to end: render tags at known 3D poses, detect them, and solve the poses back.

use nalgebra::{Isometry3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use sonoma_vision::detector::{Config, Detector, Family, ImageView};
use sonoma_vision::pose::field::FieldLayout;
use sonoma_vision::pose::{Estimator, FRC_TAG_SIZE, PoseBounds, PoseConfig, tag_corners};
use sonoma_vision::synth::{Canvas, Rng};

const W: usize = 1280;
const H: usize = 720;

fn config(layout: Option<FieldLayout>) -> PoseConfig {
    PoseConfig {
        calibration: None,
        fov_deg: 75.0,
        tag_size: FRC_TAG_SIZE,
        layout,
        robot_to_camera: Isometry3::identity(),
        max_ambiguity: 0.2,
        corner_noise_px: 1.0,
        bounds: PoseBounds::DEFAULT,
    }
}

/// The pixels the corners of a tag at `camera_to_tag` (WPILib camera frame) land on.
fn project(est: &Estimator, camera_to_tag: &Isometry3<f64>) -> [[f64; 2]; 4] {
    tag_corners(FRC_TAG_SIZE).map(|c| {
        let p = camera_to_tag * Point3::from(c);
        // WPILib camera frame (X forward, Y left, Z up) to normalised image coordinates.
        est.camera.project([-p.y / p.x, -p.z / p.x])
    })
}

/// Render the tags, blur and noise them like a real camera, and detect.
fn render_and_detect(
    est: &Estimator,
    tags: &[(u32, Isometry3<f64>)],
    seed: u64,
) -> Vec<sonoma_vision::detector::Detection> {
    let family = Family::by_name("tag36h11").unwrap();
    let mut canvas = Canvas::new(W, H, 120.0);
    for (id, pose) in tags {
        canvas.draw_tag(family, *id as usize, project(est, pose), 30.0, 220.0);
    }
    canvas.blur(0.7);
    canvas.noise(2.0, &mut Rng::new(seed));
    let gray = canvas.to_gray();
    Detector::new(family, &Config::default()).unwrap().detect(&ImageView::gray(&gray, W, H).unwrap())
}

/// A tag `distance` ahead, `bearing` degrees to the left, turned `turn` degrees about its vertical axis
/// (0: facing the camera squarely).
fn tag_at(distance: f64, bearing: f64, height: f64, turn: f64) -> Isometry3<f64> {
    let b = bearing.to_radians();
    Isometry3::from_parts(
        Translation3::new(distance * b.cos(), distance * b.sin(), height),
        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI + turn.to_radians()),
    )
}

#[test]
fn tag_poses_match_the_truth() {
    let est = Estimator::new(&config(None), W, H).unwrap();
    for (i, truth) in [
        tag_at(1.0, 0.0, 0.0, 0.0),
        tag_at(1.5, 10.0, 0.2, 30.0),
        tag_at(3.0, -15.0, -0.3, -40.0),
        tag_at(5.0, 5.0, 0.4, 20.0),
    ]
    .iter()
    .enumerate()
    {
        let detections = render_and_detect(&est, &[(2, *truth)], i as u64);
        assert_eq!(detections.len(), 1, "case {i}");
        let pose = est.tag(&detections[0]).unwrap();
        let distance = truth.translation.vector.norm();
        let position_error = (pose.camera_to_tag.translation.vector - truth.translation.vector).norm();
        let angle_error = pose.camera_to_tag.rotation.angle_to(&truth.rotation).to_degrees();
        println!(
            "case {i}: {distance:.1} m, position error {:.1} mm ({:.2}%), angle error {angle_error:.2}°, ambiguity {:.2}, {:.2} px",
            position_error * 1e3,
            100.0 * position_error / distance,
            pose.ambiguity,
            pose.error_px
        );
        assert!(position_error < 0.01 * distance, "case {i}: position off by {position_error} m");
        // Orientation from four corners is the weak part of single-tag pose, more so far away.
        assert!(angle_error < 1.0 + distance, "case {i}: orientation off by {angle_error}°");
    }
}

#[test]
fn robot_pose_from_field_tags() {
    let layout = FieldLayout::load("2026-rebuilt-welded").unwrap();
    // Two tags on one face of the field, side by side.
    let mut ids: Vec<u32> = layout.tags.keys().copied().collect();
    ids.sort();
    let (a, b) = ids
        .iter()
        .flat_map(|&i| ids.iter().map(move |&j| (i, j)))
        .find(|&(i, j)| {
            let (ti, tj) = (layout.tags[&i], layout.tags[&j]);
            i < j
                && (0.3..1.0).contains(&(ti.translation.vector - tj.translation.vector).norm())
                && ti.rotation.angle_to(&tj.rotation) < 1e-6
        })
        .unwrap();
    let robot_to_camera = Isometry3::from_parts(
        Translation3::new(0.25, 0.1, 0.6),
        UnitQuaternion::from_euler_angles(0.0, (-10.0f64).to_radians(), 0.0),
    );
    let est = Estimator::new(&PoseConfig { robot_to_camera, ..config(Some(layout.clone())) }, W, H).unwrap();

    // The robot 2.5 m out from the tags, a little to one side, turned to face them.
    let (ta, tb) = (layout.tags[&a], layout.tags[&b]);
    let out = ta.rotation * Vector3::x();
    let mid = (ta.translation.vector + tb.translation.vector) / 2.0;
    let side = Vector3::z().cross(&out);
    let robot_at = mid + out * 2.5 + side * 0.3;
    let facing = Rotation3::from_basis_unchecked(&[-out, Vector3::z().cross(&-out), Vector3::z()]);
    let robot = Isometry3::from_parts(
        Translation3::new(robot_at.x, robot_at.y, 0.0),
        UnitQuaternion::from_rotation_matrix(&facing) * UnitQuaternion::from_euler_angles(0.0, 0.0, 0.1),
    );
    let camera = robot * robot_to_camera;

    let tags: Vec<(u32, Isometry3<f64>)> = [a, b].iter().map(|&id| (id, camera.inverse() * layout.tags[&id])).collect();
    let detections = render_and_detect(&est, &tags, 7);
    assert_eq!(detections.len(), 2);
    let poses: Vec<_> = detections.iter().map(|d| est.tag(d)).collect();
    let field = est.field(&detections, &poses).unwrap().unwrap();
    let mut used = field.tags.clone();
    used.sort();
    assert_eq!(used, vec![a, b]);
    let position_error = (field.robot.translation.vector - robot.translation.vector).norm();
    let angle_error = field.robot.rotation.angle_to(&robot.rotation).to_degrees();
    println!(
        "robot: position error {:.1} mm, angle error {angle_error:.2}°, {:.2} px",
        position_error * 1e3,
        field.error_px
    );
    assert!(position_error < 0.03, "robot position off by {position_error} m");
    assert!(angle_error < 1.0, "robot orientation off by {angle_error}°");
}
