//! Pose solving: each tag's pose relative to the camera, and the robot's pose on the field.
//!
//! Poses are in WPILib's conventions throughout. The camera frame is X forward, Y left, Z up. A tag's
//! frame has +X out of its face, +Z up, and +Y to the right as you face the tag (WPILib's
//! `AprilTagFieldLayout`). The field frame is WPILib's field coordinates. Detected corners are in the
//! detector's order: the printed tag's top-left, top-right, bottom-right, bottom-left.

pub mod camera;
pub mod field;
pub mod solve;
pub mod uncertainty;

use anyhow::Result;
use nalgebra::{Isometry3, Matrix3, Matrix6, Point3, Rotation3, UnitQuaternion, Vector3, Vector6};

use crate::detector::Detection;
use camera::Intrinsics;
use field::FieldLayout;
use uncertainty::Pose2d;

/// The FRC tag size since 2024: the black square's side, 6.5 in.
pub const FRC_TAG_SIZE: f64 = 0.1651;

#[derive(Clone, Debug)]
pub struct PoseConfig {
    /// A calibration (at any resolution scaling to the camera's); None estimates one from `fov_deg`.
    pub calibration: Option<Intrinsics>,
    /// Diagonal field of view for the estimate, in degrees.
    pub fov_deg: f64,
    /// The side of the tag's black square, in metres.
    pub tag_size: f64,
    /// Where the tags are; None solves only each tag's pose relative to the camera.
    pub layout: Option<FieldLayout>,
    /// The camera's pose on the robot (WPILib's robot-to-camera transform).
    pub robot_to_camera: Isometry3<f64>,
    /// Single-tag field poses more ambiguous than this are dropped.
    pub max_ambiguity: f64,
    /// Each corner coordinate's noise, in pixels, that covariances assume.
    pub corner_noise_px: f64,
    /// Where the robot can be; poses elsewhere are dropped.
    pub bounds: PoseBounds,
}

/// Where the robot can physically be. A solution outside these is dropped, which settles many
/// single-tag ambiguities: the mirror-image pose usually has the robot tilted or off the floor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseBounds {
    /// Most metres the robot's origin may be above (or below) the floor.
    pub max_height: f64,
    /// Most degrees the robot may tilt from level (roll and pitch together).
    pub max_tilt_deg: f64,
    /// Metres beyond the field's edges the robot may be.
    pub field_margin: f64,
}

impl PoseBounds {
    pub const DEFAULT: PoseBounds = PoseBounds { max_height: 2.0, max_tilt_deg: 30.0, field_margin: 1.0 };

    /// Why the robot can't be at `robot` on `layout`'s field, if it can't.
    pub fn check(&self, robot: &Isometry3<f64>, layout: &FieldLayout) -> Result<(), String> {
        let t = robot.translation.vector;
        let m = self.field_margin;
        if !(-m..=layout.length + m).contains(&t.x) || !(-m..=layout.width + m).contains(&t.y) {
            return Err(format!("robot off the field at ({:.2}, {:.2}) m", t.x, t.y));
        }
        if t.z.abs() > self.max_height {
            return Err(format!("robot {:.2} m off the floor (max_height {})", t.z, self.max_height));
        }
        let tilt = (robot.rotation * Vector3::z()).z.clamp(-1.0, 1.0).acos().to_degrees();
        if tilt > self.max_tilt_deg {
            return Err(format!("robot tilted {tilt:.0}° (max_tilt {}; is robot_to_camera right?)", self.max_tilt_deg));
        }
        Ok(())
    }
}

impl Default for PoseBounds {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// How far the fit's valleys are traced (see [`Estimator::trace`]): until the corners fit worse than
/// the best by this many σ² (σ the corner noise), where a pose's likelihood is e^-12.5 the best's.
const TRACE_LIMIT: f64 = 25.0;

/// Most steps each way along a valley.
const TRACE_STEPS: usize = 24;

/// The fraction of each variance added back onto it (see [`uncertainty::inflate`]).
const VARIANCE_SLACK: f64 = 0.1;

/// A pose whose corners fit worse than corner noise would leave this rarely is rejected: the tags
/// can't all be where the layout says, or the camera model is off.
const MISFIT_PROBABILITY: f64 = 1e-4;

/// A tag joins a group when its corners are no farther from where the group's pose predicts them
/// than corner noise would leave them this rarely.
const AGREEMENT_PROBABILITY: f64 = 1e-3;

/// The χ² value with even `dof` degrees of freedom that's exceeded with probability `p`: where
/// `e^(-x/2) Σ_{i<dof/2} (x/2)^i / i!` falls to `p`.
fn chi2_exceeded(dof: usize, p: f64) -> f64 {
    let survival = |x: f64| {
        let (mut term, mut sum) = (1.0, 1.0);
        for i in 1..dof / 2 {
            term *= x / 2.0 / i as f64;
            sum += term;
        }
        (-x / 2.0).exp() * sum
    };
    let (mut lo, mut hi) = (0.0, 10.0 * dof as f64 + 100.0);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if survival(mid) > p { lo = mid } else { hi = mid }
    }
    hi
}

/// The plane the tags all lie on, if they do (to 2° and 2 cm): its frame in the field, with z along
/// the tags' faces' normal and the origin at `points`' centroid.
fn common_plane(tags: &[&Isometry3<f64>], points: &[Vector3<f64>]) -> Option<Isometry3<f64>> {
    let normal = tags[0].rotation * Vector3::x();
    let on_plane = tags.iter().all(|t| {
        (t.rotation * Vector3::x()).angle(&normal) < 2f64.to_radians()
            && normal.dot(&(t.translation.vector - tags[0].translation.vector)).abs() < 0.02
    });
    if !on_plane {
        return None;
    }
    let centroid = points.iter().sum::<Vector3<f64>>() / points.len() as f64;
    let x = Vector3::z().cross(&normal).try_normalize(1e-9).unwrap_or_else(|| normal.cross(&Vector3::x()).normalize());
    let basis = [x, normal.cross(&x), normal];
    Some(Isometry3::from_parts(
        centroid.into(),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_basis_unchecked(&basis)),
    ))
}

/// The camera's pose on the robot as it's configured: x, y, z in metres (forward, left, up from the
/// robot's origin), then roll, pitch, yaw in degrees. Written `x,y,z,roll,pitch,yaw`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MountPose(pub [f64; 6]);

impl MountPose {
    /// WPILib's robot-to-camera transform.
    pub fn isometry(&self) -> Isometry3<f64> {
        let [x, y, z, roll, pitch, yaw] = self.0;
        Isometry3::from_parts(
            nalgebra::Translation3::new(x, y, z),
            UnitQuaternion::from_euler_angles(roll.to_radians(), pitch.to_radians(), yaw.to_radians()),
        )
    }

    /// From six numbers, which must be finite.
    pub fn from_slice(v: &[f64]) -> Result<Self, String> {
        match <[f64; 6]>::try_from(v) {
            Ok(a) if a.iter().all(|x| x.is_finite()) => Ok(Self(a)),
            Ok(_) => Err("x,y,z,roll,pitch,yaw must be finite".into()),
            Err(_) => Err(format!("expected x,y,z,roll,pitch,yaw (6 numbers), got {}", v.len())),
        }
    }
}

impl std::str::FromStr for MountPose {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        let v: Vec<f64> =
            s.split(',').map(|p| p.trim().parse::<f64>()).collect::<Result<_, _>>().map_err(|e| e.to_string())?;
        Self::from_slice(&v)
    }
}

impl std::fmt::Display for MountPose {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let parts: Vec<String> = self.0.iter().map(f64::to_string).collect();
        f.write_str(&parts.join(","))
    }
}

/// One tag's pose relative to the camera.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TagPose {
    /// The tag's pose in the camera frame: WPILib's camera-to-tag transform.
    pub camera_to_tag: Isometry3<f64>,
    /// The other IPPE solution (the mirror image about the line of sight), if it fits at all.
    pub alternate: Option<Isometry3<f64>>,
    /// RMS corner reprojection error of `camera_to_tag`, in pixels.
    pub error_px: f64,
    /// The best solution's error over the alternate's (0-1): near 1, the two fit equally well and
    /// the tag's orientation is uncertain. 0 if there's no alternate.
    pub ambiguity: f64,
}

/// The robot's pose on the field, from the tags in view.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldPose {
    pub robot: Isometry3<f64>,
    pub camera: Isometry3<f64>,
    /// The tags used.
    pub tags: Vec<u32>,
    /// RMS corner reprojection error, in pixels.
    pub error_px: f64,
    /// The best solution's RMS error over the next best's (0-1): near 1, two poses fit about equally
    /// well. 0 with only one solution (on the field, within [`PoseBounds`]).
    pub ambiguity: f64,
    /// The robot pose's covariance on the field (see [`uncertainty`]), covering rival solutions as
    /// far as they fit.
    pub covariance: Matrix6<f64>,
    /// The standard deviations of x and y along the robot's forward and left axes, and of yaw (radians),
    /// for WPILib's `addVisionMeasurement`.
    pub std_devs: [f64; 3],
    /// Tags in view and in the layout that disagreed with the rest, and were left out: each tag's ID
    /// and why.
    pub outliers: Vec<Outlier>,
}

/// A tag left out of a pose: its ID, and why.
pub type Outlier = (u32, String);

/// A tag's corners on the field, and where they were seen: normalised, and in pixels.
type TagPoints = (Vec<Vector3<f64>>, Vec<[f64; 2]>, Vec<[f64; 2]>);

/// A detection whose tag is in the layout: the detection, its pose relative to the camera, and the
/// tag's pose on the field.
type Known<'a> = (&'a Detection, &'a TagPose, &'a Isometry3<f64>);

/// A pose sampled along a fit's valley (see [`Estimator::trace`]).
struct Sample {
    /// World to optical camera.
    pose: Isometry3<f64>,
    /// The covariance of the pose's increment (as [`solve::information`]'s), across the valley.
    covariance: Matrix6<f64>,
    weight: f64,
}

/// The increment `[r, t]` (as [`solve::apply_increment`] takes) from `a` to `b`.
fn increment_between(a: &Isometry3<f64>, b: &Isometry3<f64>) -> Vector6<f64> {
    let r = (b.rotation * a.rotation.inverse()).scaled_axis();
    let t = b.translation.vector - a.translation.vector;
    Vector6::new(r.x, r.y, r.z, t.x, t.y, t.z)
}

/// The camera's optical frame (x right, y down, z forward) in WPILib's camera frame.
fn wpilib_from_optical() -> Rotation3<f64> {
    Rotation3::from_matrix_unchecked(Matrix3::new(0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0))
}

/// A tag's frame in the solver's plane frame (x right and y down as you face the tag, z into it).
fn plane_from_tag() -> Rotation3<f64> {
    Rotation3::from_matrix_unchecked(Matrix3::new(0.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0))
}

fn rotation(r: Rotation3<f64>) -> Isometry3<f64> {
    Isometry3::from_parts(Default::default(), UnitQuaternion::from_rotation_matrix(&r))
}

/// The tag's corners in its own frame, in the detector's order.
pub fn tag_corners(size: f64) -> [Vector3<f64>; 4] {
    let h = size / 2.0;
    [Vector3::new(0.0, -h, h), Vector3::new(0.0, h, h), Vector3::new(0.0, h, -h), Vector3::new(0.0, -h, -h)]
}

pub struct Estimator {
    pub camera: Intrinsics,
    cfg: PoseConfig,
}

impl Estimator {
    /// An estimator for a camera streaming `width` × `height`.
    pub fn new(cfg: &PoseConfig, width: usize, height: usize) -> Result<Self> {
        let camera = match &cfg.calibration {
            Some(c) => c.scaled_to(width, height)?,
            None => Intrinsics::from_fov(width, height, cfg.fov_deg),
        };
        Ok(Self { camera, cfg: cfg.clone() })
    }

    /// The tag's pose relative to the camera.
    pub fn tag(&self, d: &Detection) -> Option<TagPose> {
        let corners = tag_corners(self.cfg.tag_size);
        let plane = corners.map(|c| {
            let p = plane_from_tag() * c;
            [p.x, p.y]
        });
        let image = d.corners.map(|p| self.camera.normalize(p));
        let solutions = solve::solve_planar(&plane, &image);
        let to_camera =
            |plane_pose: &Isometry3<f64>| rotation(wpilib_from_optical()) * plane_pose * rotation(plane_from_tag());
        let (best, _) = solutions.first()?;
        let camera_to_tag = to_camera(best);
        let error_px =
            self.error_px(&(rotation(wpilib_from_optical().inverse()) * camera_to_tag), &corners, &d.corners);
        // An alternate that refined onto the best pose (as when the tag faces the camera squarely) is
        // the same answer, not a rival.
        let alternate = solutions.get(1).map(|(p, _)| to_camera(p)).filter(|alt| {
            alt.rotation.angle_to(&camera_to_tag.rotation) > 1e-4
                || (alt.translation.vector - camera_to_tag.translation.vector).norm() > 1e-6
        });
        let ambiguity = match alternate {
            Some(alt) => {
                let alt_error = self.error_px(&(rotation(wpilib_from_optical().inverse()) * alt), &corners, &d.corners);
                if alt_error > 0.0 { (error_px / alt_error).min(1.0) } else { 1.0 }
            }
            None => 0.0,
        };
        Some(TagPose { camera_to_tag, alternate, error_px, ambiguity })
    }

    /// The robot's pose from the detections (with their [`Self::tag`] poses) whose tags are in the
    /// layout. Ok(None) without any such tags; Err with the reason when the tags were seen but no
    /// pose is trustworthy.
    ///
    /// Every tag is fitted jointly over every corner: each tag's two solutions seed the fit, and so
    /// does the wall's when the tags are on one wall. The distinct fits that leave the robot within
    /// [`PoseBounds`] are the candidates. The best is the pose, and the valleys through each widen its
    /// covariance as far as they fit. When several tags don't fit together (or leave no pose within the bounds), the
    /// largest group that agree is fitted instead, and the rest are left
    /// out as [`FieldPose::outliers`].
    pub fn field(&self, detections: &[Detection], tags: &[Option<TagPose>]) -> Result<Option<FieldPose>, String> {
        let Some(layout) = self.cfg.layout.as_ref() else { return Ok(None) };
        let known: Vec<Known> =
            detections.iter().zip(tags).filter_map(|(d, t)| Some((d, t.as_ref()?, layout.tags.get(&d.id)?))).collect();
        if known.is_empty() {
            return Ok(None);
        }
        // Every tag, unless they don't fit together: then the largest group that agree.
        let failure = match self.solve_tags(&known, layout) {
            Ok(pose) => return Ok(Some(pose)),
            Err(why) if known.len() == 1 => return Err(why),
            Err(why) => why,
        };
        let (inliers, outliers) = self.consensus(&known)?;
        if outliers.is_empty() {
            return Err(failure);
        }
        let mut pose = self.solve_tags(&inliers, layout)?;
        pose.outliers = outliers;
        Ok(Some(pose))
    }

    /// The largest group of `known` that agree on where the camera is, and the rest, each with why
    /// it was left out. Each tag's two solutions are a hypothesis; the tags
    /// whose corners are where the hypothesis predicts, allowing for its uncertainty
    /// ([`solve::prediction_distance`]), join it, and the group is refitted and tested again until it
    /// settles. Err when no two tags agree, or when two groups of the same size disagree: then
    /// there's no telling which tags are where the layout says.
    /// Membership is all this decides: [`Self::solve_tags`] fits the group and applies the bounds.
    fn consensus<'a>(&self, known: &[Known<'a>]) -> Result<(Vec<Known<'a>>, Vec<Outlier>), String> {
        let corners = tag_corners(self.cfg.tag_size);
        let variance = self.cfg.corner_noise_px.powi(2);
        let optical = rotation(wpilib_from_optical().inverse());
        // Each detection's corners on the field, and where they were seen (normalised, and in pixels).
        let points: Vec<TagPoints> = known
            .iter()
            .map(|(d, _, field_tag)| {
                let world = corners.iter().map(|c| (*field_tag * Point3::from(*c)).coords).collect();
                (world, d.corners.iter().map(|&p| self.camera.normalize(p)).collect(), d.corners.to_vec())
            })
            .collect();
        let gather = |members: &[usize]| {
            let world: Vec<Vector3<f64>> = members.iter().flat_map(|&i| points[i].0.clone()).collect();
            let obs: Vec<[f64; 2]> = members.iter().flat_map(|&i| points[i].1.clone()).collect();
            (world, obs)
        };
        let agrees = |i: usize, pose: &Isometry3<f64>, info: &Matrix6<f64>| {
            let (world, obs, _) = &points[i];
            solve::prediction_distance(pose, info, world, obs, self.camera.fx, self.camera.fy, variance)
                .is_some_and(|d| d < chi2_exceeded(2 * world.len(), AGREEMENT_PROBABILITY))
        };
        // Each hypothesis's group, its fitted pose and the group's squared error.
        let mut groups: Vec<(Vec<usize>, Isometry3<f64>, f64)> = Vec::new();
        for (h, (_, t, field_tag)) in known.iter().enumerate() {
            for camera_to_tag in std::iter::once(t.camera_to_tag).chain(t.alternate) {
                let mut pose = optical * (*field_tag * camera_to_tag.inverse()).inverse();
                let mut members = vec![h];
                for _ in 0..4 {
                    let (world, obs) = gather(&members);
                    if members.len() > 1 && solve::refine(&mut pose, &world, &obs).is_none() {
                        break;
                    }
                    let info = solve::information(&pose, &world, self.camera.fx, self.camera.fy) / variance;
                    let grown: Vec<usize> = (0..known.len()).filter(|&i| i == h || agrees(i, &pose, &info)).collect();
                    if grown == members {
                        break;
                    }
                    members = grown;
                }
                let (world, obs) = gather(&members);
                if solve::refine(&mut pose, &world, &obs).is_none() {
                    continue;
                }
                let pixels: Vec<[f64; 2]> = members.iter().flat_map(|&i| points[i].2.clone()).collect();
                groups.push((members, pose, self.squared_error(&pose, &world, &pixels)));
            }
        }
        let ids = |members: &[usize]| members.iter().map(|&i| known[i].0.id).collect::<Vec<u32>>();
        let all: Vec<usize> = (0..known.len()).collect();
        groups.sort_by(|a, b| b.0.len().cmp(&a.0.len()).then(a.2.total_cmp(&b.2)));
        let Some((best, pose, _)) = groups.first() else {
            return Err(format!("tags {:?}: no fit", ids(&all)));
        };
        if best.len() < 2 {
            return Err(format!("tags {:?} disagree on the pose, and can't all be where the layout says", ids(&all)));
        }
        if let Some((rival, _, _)) =
            groups.iter().find(|(g, _, _)| g.len() == best.len() && g.iter().all(|i| !best.contains(i)))
        {
            return Err(format!(
                "tags {:?} and {:?} disagree on the pose; no telling which are where the layout says",
                ids(best),
                ids(rival)
            ));
        }
        let outliers = all
            .iter()
            .filter(|i| !best.contains(i))
            .map(|&i| {
                let (world, _, pixels) = &points[i];
                let rms = self.error_px(pose, world, pixels);
                (known[i].0.id, format!("{rms:.1} px RMS off the pose tags {:?} give", ids(best)))
            })
            .collect();
        Ok((best.iter().map(|&i| known[i]).collect(), outliers))
    }

    /// The robot's pose from `known`, fitted jointly over every corner (see [`Self::field`]).
    fn solve_tags(&self, known: &[Known], layout: &FieldLayout) -> Result<FieldPose, String> {
        let corners = tag_corners(self.cfg.tag_size);
        let ids: Vec<u32> = known.iter().map(|(d, _, _)| d.id).collect();
        let fail = |why: String| Err(format!("tags {ids:?}: {why}"));

        // Every corner, in field coordinates, against where it was seen.
        let mut world = Vec::new();
        let mut pixels = Vec::new();
        for (d, _, field_tag) in known {
            for (c, p) in corners.iter().zip(&d.corners) {
                world.push((*field_tag * Point3::from(*c)).coords);
                pixels.push(*p);
            }
        }
        let obs: Vec<[f64; 2]> = pixels.iter().map(|&p| self.camera.normalize(p)).collect();
        let optical = rotation(wpilib_from_optical().inverse());
        let camera_to_robot = self.cfg.robot_to_camera.inverse();
        let robot_of = |field_to_optical: &Isometry3<f64>| field_to_optical.inverse() * optical * camera_to_robot;

        // Starting points: each tag's two solutions, and with several tags on one wall, the wall's
        // own two (a wall seen head-on can mislead every tag's solutions alike).
        let mut seeds: Vec<Isometry3<f64>> = known
            .iter()
            .flat_map(|(_, t, field_tag)| {
                std::iter::once(t.camera_to_tag)
                    .chain(t.alternate)
                    .map(move |camera_to_tag| optical * (*field_tag * camera_to_tag.inverse()).inverse())
            })
            .collect();
        let field_tags: Vec<&Isometry3<f64>> = known.iter().map(|(_, _, t)| *t).collect();
        if known.len() > 1
            && let Some(plane_to_field) = common_plane(&field_tags, &world)
        {
            let field_to_plane = plane_to_field.inverse();
            let plane: Vec<[f64; 2]> =
                world.iter().map(|w| field_to_plane * Point3::from(*w)).map(|p| [p.x, p.y]).collect();
            seeds.extend(solve::solve_planar(&plane, &obs).into_iter().map(|(pose, _)| pose * field_to_plane));
        }

        // The distinct fits from those, best first, with their corners' squared error.
        let mut fits: Vec<(Isometry3<f64>, f64)> = Vec::new();
        for mut field_to_optical in seeds {
            if solve::refine(&mut field_to_optical, &world, &obs).is_none() {
                continue;
            }
            let same = |(f, _): &(Isometry3<f64>, f64)| {
                f.rotation.angle_to(&field_to_optical.rotation) < 1e-3
                    && (f.translation.vector - field_to_optical.translation.vector).norm() < 1e-3
            };
            if !fits.iter().any(same) {
                fits.push((field_to_optical, self.squared_error(&field_to_optical, &world, &pixels)));
            }
        }
        fits.sort_by(|a, b| a.1.total_cmp(&b.1));
        let mut reasons = Vec::new();
        fits.retain(|(f, _)| self.cfg.bounds.check(&robot_of(f), layout).map_err(|e| reasons.push(e)).is_ok());
        let Some(&(best, best_sse)) = fits.first() else {
            return fail(reasons.first().cloned().unwrap_or_else(|| "no fit".into()));
        };
        let n = world.len() as f64;
        let variance = self.cfg.corner_noise_px.powi(2);
        if best_sse / variance > chi2_exceeded(2 * world.len() - 6, MISFIT_PROBABILITY) {
            return fail(format!(
                "corners fit {:.2} px RMS, too far for corner_noise_px {} (is the layout or calibration off?)",
                (best_sse / n).sqrt(),
                self.cfg.corner_noise_px
            ));
        }
        let ambiguity = match fits.get(1) {
            Some(&(_, sse)) if sse > 0.0 => (best_sse / sse).sqrt().min(1.0),
            Some(_) => 1.0,
            None => 0.0,
        };
        if known.len() == 1 && ambiguity > self.cfg.max_ambiguity {
            return fail(format!("ambiguity {ambiguity:.2} over max_ambiguity {}", self.cfg.max_ambiguity));
        }

        // The likely poses along each fit's valley, as robot poses with covariances and weights.
        let mut samples: Vec<Sample> = Vec::new();
        for (f, _) in &fits {
            self.trace(
                f,
                best_sse,
                &world,
                &obs,
                &pixels,
                &|f| self.cfg.bounds.check(&robot_of(f), layout).is_ok(),
                &mut samples,
            );
        }
        let members: Vec<(Isometry3<f64>, Matrix6<f64>, f64)> = samples
            .iter()
            .map(|s| {
                let camera_cov = uncertainty::camera_in_world(&s.pose, &s.covariance);
                let field_to_camera = s.pose.inverse();
                let robot = robot_of(&s.pose);
                (
                    robot,
                    uncertainty::attached(&field_to_camera, &(field_to_camera.inverse() * robot), &camera_cov),
                    s.weight,
                )
            })
            .collect();
        let robot = robot_of(&best);
        let covariance = uncertainty::inflate(&uncertainty::mixture(&robot, &members), VARIANCE_SLACK);
        let floor: Vec<(Pose2d, f64)> = members.iter().map(|(p, c, w)| (Pose2d::project(p, c), *w)).collect();
        let mut pose2d = Pose2d::project(&robot, &Matrix6::zeros());
        pose2d.covariance = uncertainty::inflate(&pose2d.mixture(&floor), VARIANCE_SLACK);

        Ok(FieldPose {
            robot,
            camera: best.inverse() * optical,
            tags: ids,
            error_px: (best_sse / n).sqrt(),
            ambiguity,
            covariance,
            std_devs: pose2d.local_std_devs(),
            outliers: Vec::new(),
        })
    }

    /// Sample the poses along the fit's valley through `start` (world to optical), appending them to
    /// `samples`: from `start`, step both ways along the direction the corners constrain least,
    /// re-fitting the other five directions at each step, until the corners fit worse than `best_sse`
    /// by [`TRACE_LIMIT`] σ², or the pose leaves `allowed`. Each sample carries its covariance with
    /// the traced direction conditioned out (the trace itself covers that), and a weight: its
    /// likelihood times its spacing along the valley. Where a valley is a narrow bowl, the samples
    /// make up its Gaussian; where it's long and flat (tags seen head-on, from afar), they follow it.
    /// A start within a step of an earlier sample is in a valley already traced, and adds nothing.
    #[allow(clippy::too_many_arguments)]
    fn trace(
        &self,
        start: &Isometry3<f64>,
        best_sse: f64,
        world: &[Vector3<f64>],
        obs: &[[f64; 2]],
        pixels: &[[f64; 2]],
        allowed: &dyn Fn(&Isometry3<f64>) -> bool,
        samples: &mut Vec<Sample>,
    ) {
        let variance = self.cfg.corner_noise_px.powi(2);
        let information = |p: &Isometry3<f64>| solve::information(p, world, self.camera.fx, self.camera.fy) / variance;
        // The least constrained direction and its standard deviation, and the covariance of the other
        // five directions given it.
        let local = |info: &Matrix6<f64>| -> Option<(Vector6<f64>, f64, Matrix6<f64>)> {
            let eig = info.symmetric_eigen();
            let i = eig.eigenvalues.imin();
            let v: Vector6<f64> = eig.eigenvectors.column(i).into_owned();
            let p = info.try_inverse()?;
            let pv = p * v;
            Some((v, 1.0 / eig.eigenvalues[i].max(1e-12).sqrt(), p - pv * pv.transpose() / v.dot(&pv)))
        };
        let near_traced = |pose: &Isometry3<f64>, info: &Matrix6<f64>| {
            samples.iter().any(|s| {
                let d = increment_between(&s.pose, pose);
                d.dot(&(info * d)) < 1.0
            })
        };
        let start_info = information(start);
        let Some((v0, h0, cov0)) = local(&start_info) else { return };
        if near_traced(start, &start_info) {
            return;
        }
        let sse0 = self.squared_error(start, world, pixels);
        let weight = |sse: f64, h: f64| (-(sse - best_sse) / (2.0 * variance)).exp() * h;
        samples.push(Sample { pose: *start, covariance: cov0, weight: weight(sse0, h0) });
        for sign in [1.0, -1.0] {
            let (mut pose, mut prev) = (*start, v0 * sign);
            let (mut v, mut h) = (prev, h0);
            for _ in 0..TRACE_STEPS {
                let mut next = solve::apply_increment(&pose, &(v * h));
                if solve::refine_across(&mut next, world, obs, &v).is_none() || !allowed(&next) {
                    break;
                }
                let sse = self.squared_error(&next, world, pixels);
                if (sse - best_sse) / variance > TRACE_LIMIT {
                    break;
                }
                let info = information(&next);
                let Some((nv, nh, cov)) = local(&info) else { break };
                samples.push(Sample { pose: next, covariance: cov, weight: weight(sse, nh) });
                (pose, prev) = (next, v);
                (v, h) = (if nv.dot(&prev) < 0.0 { -nv } else { nv }, nh);
            }
        }
    }

    /// The sum of squared pixel distances between `points` taken into the optical frame by
    /// `to_optical`, projected, and the detected `pixels`.
    fn squared_error(&self, to_optical: &Isometry3<f64>, points: &[Vector3<f64>], pixels: &[[f64; 2]]) -> f64 {
        self.error_px(to_optical, points, pixels).powi(2) * points.len() as f64
    }

    /// RMS pixel distance between `points` taken into the optical frame by `to_optical`, projected,
    /// and the detected `pixels`.
    fn error_px(&self, to_optical: &Isometry3<f64>, points: &[Vector3<f64>], pixels: &[[f64; 2]]) -> f64 {
        let sum: f64 = points
            .iter()
            .zip(pixels)
            .map(|(p, px)| {
                let c = to_optical * Point3::from(*p);
                let [u, v] = self.camera.project([c.x / c.z, c.y / c.z]);
                (u - px[0]).powi(2) + (v - px[1]).powi(2)
            })
            .sum();
        (sum / points.len() as f64).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Translation3;

    fn config(layout: Option<FieldLayout>) -> PoseConfig {
        PoseConfig {
            calibration: None,
            fov_deg: 70.0,
            tag_size: FRC_TAG_SIZE,
            layout,
            robot_to_camera: Isometry3::identity(),
            max_ambiguity: 0.2,
            corner_noise_px: 1.0,
            bounds: PoseBounds::DEFAULT,
        }
    }

    /// Where `camera_to_tag` puts the tag's corners in the image.
    fn corners_px(est: &Estimator, camera_to_tag: &Isometry3<f64>) -> [[f64; 2]; 4] {
        let optical = rotation(wpilib_from_optical().inverse()) * camera_to_tag;
        tag_corners(FRC_TAG_SIZE).map(|c| {
            let p = optical * Point3::from(c);
            est.camera.project([p.x / p.z, p.y / p.z])
        })
    }

    fn detection(id: u32, corners: [[f64; 2]; 4]) -> Detection {
        Detection { id, hamming: 0, decision_margin: 60.0, corners }
    }

    #[test]
    fn a_tag_straight_ahead_faces_the_camera() {
        let est = Estimator::new(&config(None), 640, 480).unwrap();
        // 2 m ahead, facing the camera: its +X (out of its face) points back at us, so it's yawed 180°.
        let truth = Isometry3::from_parts(
            Translation3::new(2.0, 0.0, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
        );
        let px = corners_px(&est, &truth);
        // Upright and facing us, the top-left corner is up and to the left of centre.
        assert!(px[0][0] < 319.5 && px[0][1] < 239.5, "{px:?}");
        let pose = est.tag(&detection(1, px)).unwrap();
        assert!((pose.camera_to_tag.translation.vector - truth.translation.vector).norm() < 1e-6);
        assert!(pose.camera_to_tag.rotation.angle_to(&truth.rotation) < 1e-6);
        assert!(pose.error_px < 1e-6);
    }

    #[test]
    fn field_pose_from_one_and_two_tags() {
        let layout = FieldLayout::load("2026-rebuilt-welded").unwrap();
        // Two tags that face the same way and sit close together.
        let (a, b) = {
            let tags = &layout.tags;
            let mut ids: Vec<u32> = tags.keys().copied().collect();
            ids.sort();
            let mut pair = None;
            'outer: for &i in &ids {
                for &j in &ids {
                    let (ti, tj) = (tags[&i], tags[&j]);
                    let close = (ti.translation.vector - tj.translation.vector).norm();
                    if i < j && (0.3..2.0).contains(&close) && ti.rotation.angle_to(&tj.rotation) < 1e-6 {
                        pair = Some((i, j));
                        break 'outer;
                    }
                }
            }
            pair.unwrap()
        };
        let robot_to_camera =
            Isometry3::from_parts(Translation3::new(0.3, -0.1, 0.5), UnitQuaternion::from_euler_angles(0.0, -0.2, 0.0));
        let est = Estimator::new(&PoseConfig { robot_to_camera, ..config(Some(layout.clone())) }, 1280, 720).unwrap();
        // Put the camera 3 m out in front of tag a, looking back at it.
        let ta = layout.tags[&a];
        let out = ta.rotation * Vector3::x();
        let eye = ta.translation.vector + out * 3.0 + Vector3::new(0.0, 0.0, -0.3);
        // Looking back at it (+X along -out), level (+Z up).
        let forward = -out;
        let basis = [forward, Vector3::z().cross(&forward), Vector3::z()];
        let camera = Isometry3::from_parts(
            Translation3::from(eye),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_basis_unchecked(&basis)),
        );
        assert!(((camera.rotation * Vector3::x()) - -out).norm() < 1e-9, "camera looks at the tags");
        let robot = camera * robot_to_camera.inverse();

        let seen: Vec<Detection> =
            [a, b].iter().map(|&id| detection(id, corners_px(&est, &(camera.inverse() * layout.tags[&id])))).collect();
        let poses: Vec<Option<TagPose>> = seen.iter().map(|d| est.tag(d)).collect();

        let two = est.field(&seen, &poses).unwrap().unwrap();
        assert_eq!(two.tags, vec![a, b]);
        assert!((two.robot.translation.vector - robot.translation.vector).norm() < 1e-6, "{two:?}");
        assert!(two.robot.rotation.angle_to(&robot.rotation) < 1e-6);

        let one = est.field(&seen[..1], &poses[..1]).unwrap().unwrap();
        assert!((one.robot.translation.vector - robot.translation.vector).norm() < 1e-6);
        // Two tags pin the robot down better than one, and the standard deviations say so.
        assert!(two.std_devs.iter().zip(one.std_devs).all(|(t, o)| *t < o), "{:?} vs {:?}", two.std_devs, one.std_devs);
        assert!(one.std_devs.iter().all(|s| s.is_finite() && *s > 0.0));
        // Tags not in the layout don't count.
        assert_eq!(est.field(&[detection(99, seen[0].corners)], &poses[..1]), Ok(None));

        // A corner 15 px from where the rest put it. With two tags, they disagree, and there's no
        // telling which is right; alone, no pose fits that tag well enough to publish.
        let mut bent = seen.clone();
        bent[0].corners[0][0] += 15.0;
        let bent_poses: Vec<Option<TagPose>> = bent.iter().map(|d| est.tag(d)).collect();
        let why = est.field(&bent, &bent_poses).unwrap_err();
        assert!(why.contains("disagree"), "{why}");
        let why = est.field(&bent[..1], &bent_poses[..1]).unwrap_err();
        assert!(why.contains("corners fit"), "{why}");

        // A robot that can't be off the floor at all: the pose is dropped, and says why.
        let grounded = PoseConfig {
            robot_to_camera,
            bounds: PoseBounds { max_height: 0.0, ..PoseBounds::DEFAULT },
            ..config(Some(layout.clone()))
        };
        let est = Estimator::new(&grounded, 1280, 720).unwrap();
        let why = est.field(&seen, &poses).unwrap_err();
        assert!(why.contains("off the floor"), "{why}");
    }
}
