//! Are the published covariances honest? Project tags exactly, add Gaussian corner noise of the
//! size the estimator assumes, solve, and check how often the truth falls outside the reported
//! uncertainty. A calibrated covariance leaves the truth outside its 95% region 5% of the time,
//! and its normalised squared error (NEES) averages the number of dimensions.

use std::collections::HashMap;

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use sonoma_vision::detector::Detection;
use sonoma_vision::pose::field::FieldLayout;
use sonoma_vision::pose::uncertainty::difference;
use sonoma_vision::pose::{Estimator, FRC_TAG_SIZE, PoseBounds, PoseConfig, tag_corners};
use sonoma_vision::synth::Rng;

/// χ² 95th percentiles for 3 and 6 degrees of freedom.
const CHI2_95_3: f64 = 7.815;
const CHI2_95_6: f64 = 12.592;

fn gaussian(rng: &mut Rng) -> f64 {
    let (u, v) = (rng.unit().max(1e-300), rng.unit());
    (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
}

/// A wall at x = 8 facing -X with tags at `spots` (y, z), and a free-standing tag elsewhere for
/// scenes with depth.
fn layout(spots: &[(f64, f64)], extra: Option<Isometry3<f64>>) -> FieldLayout {
    let facing = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI);
    let mut tags: HashMap<u32, Isometry3<f64>> = spots
        .iter()
        .enumerate()
        .map(|(i, &(y, z))| (i as u32 + 1, Isometry3::from_parts(Translation3::new(8.0, y, z), facing)))
        .collect();
    if let Some(pose) = extra {
        tags.insert(99, pose);
    }
    FieldLayout { name: "test".into(), tags, length: 16.5, width: 8.0 }
}

/// A camera at (x, y, z) on the field, level, looking at `target` with a small error.
fn camera_at(eye: Vector3<f64>, target: Vector3<f64>, rng: &mut Rng) -> Isometry3<f64> {
    let to = target - eye;
    let yaw = to.y.atan2(to.x) + rng.range(-0.15, 0.15);
    let pitch = -(to.z.atan2(to.xy().norm())) + rng.range(-0.05, 0.05);
    Isometry3::from_parts(Translation3::from(eye), UnitQuaternion::from_euler_angles(0.0, pitch, yaw))
}

struct Tally {
    n: usize,
    nees6: f64,
    outside6: usize,
    outside3: usize,
    axis_outside: [usize; 3],
    rejected: usize,
    /// Frames with a tag wrongly left out (every tag here is where the layout says).
    outlier_frames: usize,
    seconds: f64,
}

impl Tally {
    fn new() -> Self {
        Self {
            n: 0,
            nees6: 0.0,
            outside6: 0,
            outside3: 0,
            axis_outside: [0; 3],
            rejected: 0,
            outlier_frames: 0,
            seconds: 0.0,
        }
    }

    fn report(&self, name: &str) -> String {
        let n = (self.n - self.rejected).max(1) as f64;
        format!(
            "{name:<28} n {:4}  NEES6 {:6.2}  out95 6D {:5.1}%  3D {:5.1}%  per-axis {:4.1}/{:4.1}/{:4.1}%  rejected {}  outliers {}  {:.3} ms",
            self.n,
            self.nees6 / n,
            100.0 * self.outside6 as f64 / n,
            100.0 * self.outside3 as f64 / n,
            100.0 * self.axis_outside[0] as f64 / n,
            100.0 * self.axis_outside[1] as f64 / n,
            100.0 * self.axis_outside[2] as f64 / n,
            self.rejected,
            self.outlier_frames,
            1e3 * self.seconds / self.n.max(1) as f64
        )
    }
}

/// Solve `trials` noisy views of `layout`'s tags from cameras placed by `place`, and tally how the
/// reported uncertainty covers the truth.
fn run(
    layout: FieldLayout,
    noise_px: f64,
    trials: usize,
    seed: u64,
    place: impl FnMut(&mut Rng) -> Isometry3<f64>,
) -> Tally {
    run_believing(layout.clone(), layout, noise_px, trials, seed, place).0
}

/// As [`run`], with the estimator told `believed` while the tags are really where `layout` says.
/// Also returns, per tag ID, the frames in which exactly that tag was left out as an outlier.
fn run_believing(
    believed: FieldLayout,
    layout: FieldLayout,
    noise_px: f64,
    trials: usize,
    seed: u64,
    mut place: impl FnMut(&mut Rng) -> Isometry3<f64>,
) -> (Tally, HashMap<u32, usize>) {
    let mut caught: HashMap<u32, usize> = HashMap::new();
    let cfg = PoseConfig {
        calibration: None,
        fov_deg: 70.0,
        tag_size: FRC_TAG_SIZE,
        layout: Some(believed),
        robot_to_camera: Isometry3::identity(),
        // Every single-tag pose, however ambiguous: its covariance has to cover that.
        max_ambiguity: 1.0,
        corner_noise_px: noise_px,
        bounds: PoseBounds::DEFAULT,
    };
    let est = Estimator::new(&cfg, 1280, 720).unwrap();
    let corners = tag_corners(FRC_TAG_SIZE);
    let mut rng = Rng::new(seed);
    let mut tally = Tally::new();
    let mut ids: Vec<u32> = layout.tags.keys().copied().collect();
    ids.sort();
    while tally.n < trials {
        let camera = place(&mut rng);
        // Only where a robot can be: elsewhere, the bounds rightly drop the truth.
        if cfg.bounds.check(&camera, &layout).is_err() {
            continue;
        }
        let mut detections = Vec::new();
        for &id in &ids {
            let to_tag = camera.inverse() * layout.tags[&id];
            let mut px = [[0.0; 2]; 4];
            let mut visible = true;
            for (k, c) in corners.iter().enumerate() {
                let p = to_tag * Point3::from(*c);
                if p.x < 0.3 {
                    visible = false;
                    break;
                }
                let [u, v] = est.camera.project([-p.y / p.x, -p.z / p.x]);
                visible &= (0.0..1279.0).contains(&u) && (0.0..719.0).contains(&v);
                px[k] = [u + noise_px * gaussian(&mut rng), v + noise_px * gaussian(&mut rng)];
            }
            // The tag must face the camera.
            let normal = to_tag.rotation * Vector3::x();
            visible &= normal.dot(&to_tag.translation.vector) < 0.0;
            if visible {
                detections.push(Detection { id, hamming: 0, decision_margin: 60.0, corners: px });
            }
        }
        if detections.is_empty() {
            continue;
        }
        let t0 = std::time::Instant::now();
        let poses: Vec<_> = detections.iter().map(|d| est.tag(d)).collect();
        let solved = est.field(&detections, &poses);
        tally.seconds += t0.elapsed().as_secs_f64();
        tally.n += 1;
        let field = match solved {
            Ok(Some(f)) => f,
            Ok(None) => unreachable!("tags in view"),
            Err(_) => {
                tally.rejected += 1;
                continue;
            }
        };
        tally.outlier_frames += usize::from(!field.outliers.is_empty());
        if let [(id, _)] = field.outliers.as_slice() {
            *caught.entry(*id).or_default() += 1;
        }
        let d = difference(&field.robot, &camera);
        let nees6 = (d.transpose() * field.covariance.try_inverse().unwrap() * d)[0];
        tally.nees6 += nees6;
        tally.outside6 += usize::from(nees6 > CHI2_95_6);
        // The floor pose: errors along the robot's own forward and left axes, and yaw.
        let (_, _, yaw) = field.robot.rotation.euler_angles();
        let (_, _, true_yaw) = camera.rotation.euler_angles();
        let to_local = Rotation3::from_euler_angles(0.0, 0.0, -yaw);
        let e = to_local * Vector3::new(d[0], d[1], 0.0);
        let dyaw = (true_yaw - yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
        let local = [e.x, e.y, dyaw];
        let [sx, sy, syaw] = field.std_devs;
        let nees3 = (Matrix3::from_diagonal(&Vector3::new(sx * sx, sy * sy, syaw * syaw)).try_inverse().unwrap()
            * Vector3::from(local))
        .dot(&Vector3::from(local));
        tally.outside3 += usize::from(nees3 > CHI2_95_3);
        for (i, (err, s)) in local.iter().zip([sx, sy, syaw]).enumerate() {
            tally.axis_outside[i] += usize::from(err.abs() > 1.96 * s);
        }
    }
    (tally, caught)
}

/// Where each trial puts the camera.
type Placer = Box<dyn FnMut(&mut Rng) -> Isometry3<f64>>;

/// A camera `distance` from (8, y0, z0) along a direction `angle` degrees off the wall's normal.
fn in_front(rng: &mut Rng, y0: f64, z0: f64, distance: (f64, f64), angle: f64) -> Isometry3<f64> {
    let d = rng.range(distance.0, distance.1);
    let a = rng.range(-angle, angle).to_radians();
    let eye = Vector3::new(8.0 - d * a.cos(), y0 + d * a.sin(), rng.range(0.3, 1.0));
    camera_at(eye, Vector3::new(8.0, y0, z0), rng)
}

#[test]
fn covariances_cover_the_truth() {
    let trials = 400;
    let mut lines = Vec::new();
    let mut worst: f64 = 0.0;
    let mut worst_axis: f64 = 0.0;
    for noise in [0.3, 1.0] {
        let scenes: Vec<(&str, FieldLayout, Placer)> = vec![
            (
                "one tag, 1-4 m, ±45°",
                layout(&[(4.0, 1.0)], None),
                Box::new(|r: &mut Rng| in_front(r, 4.0, 1.0, (1.0, 4.0), 45.0)),
            ),
            (
                "one tag, 3-6 m, head-on",
                layout(&[(4.0, 1.0)], None),
                Box::new(|r: &mut Rng| in_front(r, 4.0, 1.0, (3.0, 6.0), 8.0)),
            ),
            (
                "wall of 3, 2-6 m, ±45°",
                layout(&[(3.5, 1.0), (4.0, 1.4), (4.6, 0.9)], None),
                Box::new(|r: &mut Rng| in_front(r, 4.0, 1.1, (2.0, 6.0), 45.0)),
            ),
            (
                "wall of 2, 4-7 m, head-on",
                layout(&[(3.8, 1.0), (4.2, 1.0)], None),
                Box::new(|r: &mut Rng| in_front(r, 4.0, 1.0, (4.0, 7.0), 8.0)),
            ),
            (
                "tags at two depths, 2-5 m",
                layout(
                    &[(3.5, 1.0)],
                    Some(Isometry3::from_parts(
                        Translation3::new(6.5, 4.6, 0.8),
                        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI * 0.85),
                    )),
                ),
                Box::new(|r: &mut Rng| in_front(r, 4.0, 1.0, (3.5, 6.0), 25.0)),
            ),
        ];
        for (i, (name, layout, place)) in scenes.into_iter().enumerate() {
            let t = run(layout, noise, trials, 1000 + i as u64, place);
            let label = format!("{name} σ{noise}");
            lines.push(t.report(&label));
            let accepted = (t.n - t.rejected).max(1) as f64;
            worst = worst.max(t.outside6 as f64 / accepted);
            worst_axis = worst_axis.max(t.axis_outside.iter().copied().max().unwrap() as f64 / accepted);
        }
    }
    println!("{}", lines.join("\n"));
    // Some slack for scenes no Gaussian fits well (tags seen head-on from afar), but nowhere near the
    // 15-20% a covariance from the best fit alone leaves outside there. The per-axis figures are the
    // standard deviations robot code passes to addVisionMeasurement.
    assert!(worst < 0.12, "6D covariances overconfident:\n{}", lines.join("\n"));
    assert!(worst_axis < 0.12, "standard deviations overconfident:\n{}", lines.join("\n"));
}

/// A layout that's wrong about one tag (`id`, `moved` metres along the wall): the estimator is
/// told this one while the frames show the true layout.
fn moved(layout: &FieldLayout, id: u32, moved: f64) -> FieldLayout {
    let mut wrong = layout.clone();
    let tag = wrong.tags.get_mut(&id).unwrap();
    tag.translation.vector.y += moved;
    wrong
}

#[test]
fn a_misplaced_tag_is_left_out() {
    let trials = 300;
    let mut lines = Vec::new();
    let four = layout(
        &[(3.4, 1.0), (4.0, 1.4), (4.6, 0.9)],
        Some(Isometry3::from_parts(
            Translation3::new(6.5, 4.8, 0.8),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI * 0.85),
        )),
    );
    let three = layout(&[(3.4, 1.0), (4.0, 1.4), (4.6, 0.9)], None);
    let two = layout(&[(3.6, 1.0), (4.4, 1.2)], None);
    let place = |r: &mut Rng| in_front(r, 4.0, 1.1, (2.5, 5.0), 30.0);
    let mut check = |name: &str, truth: &FieldLayout, wrong: u32, by: f64, expect_caught: f64| {
        let (t, caught) = run_believing(moved(truth, wrong, by), truth.clone(), 1.0, trials, 7, place);
        let accepted = (t.n - t.rejected).max(1) as f64;
        let rate = *caught.get(&wrong).unwrap_or(&0) as f64 / t.n as f64;
        let others: usize = caught.iter().filter(|(id, _)| **id != wrong).map(|(_, n)| n).sum();
        lines.push(format!(
            "{} | caught tag {wrong} in {:.1}% of frames, another tag in {others}",
            t.report(name),
            100.0 * rate
        ));
        assert!(rate >= expect_caught, "{name}: tag {wrong} caught in only {:.1}%\n{}", 100.0 * rate, lines.join("\n"));
        // Left out or not, the published poses stay honest.
        let worst = t.axis_outside.iter().copied().max().unwrap() as f64 / accepted;
        assert!(worst < 0.12, "{name}: standard deviations overconfident\n{}", lines.join("\n"));
        t
    };
    check("4 tags, one 0.5 m off", &four, 2, 0.5, 0.95);
    check("3 tags, one 0.5 m off", &three, 2, 0.5, 0.95);
    check("3 tags, one 0.15 m off", &three, 3, 0.15, 0.5);
    // Two tags that disagree: no telling which is wrong, so no pose.
    let t = check("2 tags, one 0.5 m off", &two, 2, 0.5, 0.0);
    lines.push(format!("2 tags: {} of {} frames rejected", t.rejected, t.n));
    println!("{}", lines.join("\n"));
    assert!(t.rejected as f64 >= 0.95 * t.n as f64, "{}", lines.join("\n"));
}
