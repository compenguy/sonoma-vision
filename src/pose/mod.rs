//! Pose solving: each tag's pose relative to the camera, and the robot's pose on the field.
//!
//! Poses are in WPILib's conventions throughout. The camera frame is X forward, Y left, Z up. A tag's
//! frame has +X out of its face, +Z up, and +Y to the right as you face the tag (WPILib's
//! `AprilTagFieldLayout`). The field frame is WPILib's field coordinates. Detected corners are in the
//! detector's order: the printed tag's top-left, top-right, bottom-right, bottom-left.

pub mod camera;
pub mod field;
pub mod solve;

use anyhow::Result;
use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, UnitQuaternion, Vector3};

use crate::detector::Detection;
use camera::Intrinsics;
use field::FieldLayout;

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
    /// The single tag's ambiguity, or 0 with several tags (they constrain the pose fully).
    pub ambiguity: f64,
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
    /// layout: jointly over every corner when there are several, from the tag's pose when there's one.
    pub fn field(&self, detections: &[Detection], tags: &[Option<TagPose>]) -> Option<FieldPose> {
        let layout = self.cfg.layout.as_ref()?;
        let corners = tag_corners(self.cfg.tag_size);
        let known: Vec<(&Detection, &TagPose, &Isometry3<f64>)> =
            detections.iter().zip(tags).filter_map(|(d, t)| Some((d, t.as_ref()?, layout.tags.get(&d.id)?))).collect();
        let field_to_camera =
            |field_tag: &Isometry3<f64>, camera_to_tag: &Isometry3<f64>| field_tag * camera_to_tag.inverse();
        let finish = |camera: Isometry3<f64>, tags: Vec<u32>, error_px, ambiguity| FieldPose {
            robot: camera * self.cfg.robot_to_camera.inverse(),
            camera,
            tags,
            error_px,
            ambiguity,
        };
        match known.as_slice() {
            [] => None,
            [(d, t, field_tag)] => (t.ambiguity <= self.cfg.max_ambiguity)
                .then(|| finish(field_to_camera(field_tag, &t.camera_to_tag), vec![d.id], t.error_px, t.ambiguity)),
            _ => {
                // Every corner, in field coordinates, against where it was seen.
                let mut world = Vec::new();
                let mut pixels = Vec::new();
                for (d, _, field_tag) in &known {
                    for (c, p) in corners.iter().zip(&d.corners) {
                        world.push((*field_tag * Point3::from(*c)).coords);
                        pixels.push(*p);
                    }
                }
                let obs: Vec<[f64; 2]> = pixels.iter().map(|&p| self.camera.normalize(p)).collect();
                let optical = rotation(wpilib_from_optical().inverse());
                // Start from each tag's solutions (both, as one may be the mirror image) and keep the
                // best fit after refining.
                let best = known
                    .iter()
                    .flat_map(|(_, t, field_tag)| {
                        std::iter::once(t.camera_to_tag).chain(t.alternate).map(|c| field_to_camera(field_tag, &c))
                    })
                    .filter_map(|camera| {
                        let mut field_to_optical = optical * camera.inverse();
                        let cost = solve::refine(&mut field_to_optical, &world, &obs)?;
                        Some((field_to_optical, cost))
                    })
                    .min_by(|a, b| a.1.total_cmp(&b.1))?;
                let field_to_optical = best.0;
                let error_px = self.error_px(&field_to_optical, &world, &pixels);
                let camera = (optical.inverse() * field_to_optical).inverse();
                Some(finish(camera, known.iter().map(|(d, _, _)| d.id).collect(), error_px, 0.0))
            }
        }
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

        let two = est.field(&seen, &poses).unwrap();
        assert_eq!(two.tags, vec![a, b]);
        assert!((two.robot.translation.vector - robot.translation.vector).norm() < 1e-6, "{two:?}");
        assert!(two.robot.rotation.angle_to(&robot.rotation) < 1e-6);

        let one = est.field(&seen[..1], &poses[..1]).unwrap();
        assert!((one.robot.translation.vector - robot.translation.vector).norm() < 1e-6);
        // Tags not in the layout don't count.
        assert!(est.field(&[detection(99, seen[0].corners)], &poses[..1]).is_none());
    }
}
