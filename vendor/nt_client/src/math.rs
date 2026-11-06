//! Various `wpimath` data structures.

#[cfg(feature = "protobuf")]
use crate::protobuf::{ProtobufData, controller::*, geometry2d::*, geometry3d::*, kinematics::*, plant::*, spline::*, system::*, trajectory::*, wpimath::*};
#[cfg(feature = "protobuf")]
use nt_client_macros::ProtobufData;

#[cfg(feature = "struct")]
use crate::{schema::PublishSchemaError, r#struct::{StructData, StructSchema, byte::{ByteBuffer, ByteReader}}};
#[cfg(feature = "struct")]
use nt_client_macros::StructData;

/// Feedforward constants that model a simple arm.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double ks;double kg;double kv;double ka;double dt"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufArmFeedforward))
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmFeedforward {
    /// The static gain in volts.
    #[cfg_attr(feature = "protobuf", protobuf(field(ks)))]
    pub k_s: f64,
    /// The gravity gain in volts.
    #[cfg_attr(feature = "protobuf", protobuf(field(kg)))]
    pub k_g: f64,
    /// The velocity gain in V/rad/s.
    #[cfg_attr(feature = "protobuf", protobuf(field(kv)))]
    pub k_v: f64,
    /// The acceleration gain in V/rad/s².
    #[cfg_attr(feature = "protobuf", protobuf(field(ka)))]
    pub k_a: f64,
    /// The period in seconds.
    #[cfg_attr(feature = "protobuf", protobuf(field(dt)))]
    pub d_t: f64,
}

/// Robot chassis speeds.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double vx;double vy;double omega"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufChassisSpeeds)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChassisSpeeds {
    /// The x velocity in m/s.
    #[cfg_attr(feature = "protobuf", protobuf(field(vx)))]
    pub velocity_x: f64,
    /// The y velocity in m/s.
    #[cfg_attr(feature = "protobuf", protobuf(field(vy)))]
    pub velocity_y: f64,
    /// The angular velocity in rad/s.
    #[cfg_attr(feature = "protobuf", protobuf(field(omega)))]
    pub omega: f64,
}

/// Represents a hermite spline of degree 3.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double xInitial[2];double xFinal[2];double yInitial[2];double yFinal[2]"),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubicHermiteSpline {
    /// The control vector for the initial point in the x dimension.
    pub x_initial_control_vector: [f64; 2],
    /// The control vector for the final point in the x dimension.
    pub x_final_control_vector: [f64; 2],
    /// The control vector for the initial point in the y dimension.
    pub y_initial_control_vector: [f64; 2],
    /// The control vector for the final point in the y dimension.
    pub y_final_control_vector: [f64; 2],
}

#[cfg(feature = "protobuf")]
impl ProtobufData for CubicHermiteSpline {
    type Proto = ProtobufCubicHermiteSpline;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            x_initial_control_vector: proto.x_initial.try_into().ok()?,
            x_final_control_vector: proto.x_final.try_into().ok()?,
            y_initial_control_vector: proto.y_initial.try_into().ok()?,
            y_final_control_vector: proto.y_final.try_into().ok()?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            x_initial: self.x_initial_control_vector.into(),
            x_final: self.x_final_control_vector.into(),
            y_initial: self.y_initial_control_vector.into(),
            y_final: self.y_final_control_vector.into(),
            ..Default::default()
        }
    }
}

/// Constants for a DC motor.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double ks;double kv;double ka;double dt"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufDCMotor)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DCMotor {
    /// Voltage at which the motor constants were measured in volts.
    pub nominal_voltage: f64,
    /// Torque when stalled in newton meters.
    pub stall_torque: f64,
    /// Current draw when stalled in amps.
    pub stall_current: f64,
    /// Current draw under no load in amps.
    pub free_current: f64,
    /// Angular velocity under no load in rad/s.
    pub free_speed: f64,
}

/// Feedforward constants that model a differential drive drivetrain.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double kVLinear;double kALinear;double kVAngular;double kAAngular"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufDifferentialDriveFeedforward)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDriveFeedforward {
    /// The linear velocity gain in V/(m/s).
    #[cfg_attr(feature = "protobuf", protobuf(field(kv_linear)))]
    pub velocity_linear: f64,
    /// The linear acceleration gain in V/(m/s²).
    #[cfg_attr(feature = "protobuf", protobuf(field(ka_linear)))]
    pub acceleration_linear: f64,
    /// The angular velocity gain in V/(rad/s).
    #[cfg_attr(feature = "protobuf", protobuf(field(kv_angular)))]
    pub velocity_angular: f64,
    /// The angular acceleration gain in V/(rad/s²).
    #[cfg_attr(feature = "protobuf", protobuf(field(ka_angular)))]
    pub acceleration_angular: f64,
}

/// Kinematics for a differential drive.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double track_width"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufDifferentialDriveKinematics)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDriveKinematics {
    /// The differential drive track width in meters.
    pub track_width: f64,
}

/// Represents wheel positions for a differential drive drivetrain.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double left;double right")
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufDifferentialDriveWheelPositions)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDriveWheelPositions {
    /// Distance measured by the left side.
    pub left: f64,
    /// Distance measured by the right side.
    pub right: f64,
}

/// Represents the wheel speeds for a differential drive drivetrain.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double left;double right"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufDifferentialDriveWheelSpeeds)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDriveWheelSpeeds {
    /// Speed of the left side of the robot in m/s.
    pub left: f64,
    /// Speed of the right side of the robot in m/s.
    pub right: f64,
}

/// Represents the motor voltages for a differential drive drivetrain.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double left;double right"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufDifferentialDriveWheelVoltages)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDriveWheelVoltages {
    /// Left wheel voltage.
    pub left: f64,
    /// Right wheel voltage.
    pub right: f64,
}

/// Feedforward constants that model a simple elevator.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double ks;double kg;double kv;double ka;double dt"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufElevatorFeedforward)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevatorFeedforward {
    /// The static gain in volts.
    #[cfg_attr(feature = "protobuf", protobuf(field(ks)))]
    pub k_s: f64,
    /// The gravity gain in volts.
    #[cfg_attr(feature = "protobuf", protobuf(field(kg)))]
    pub k_g: f64,
    /// The velocity gain in V/(m/s).
    #[cfg_attr(feature = "protobuf", protobuf(field(kv)))]
    pub k_v: f64,
    /// The acceleration gain in V/(m/s²).
    #[cfg_attr(feature = "protobuf", protobuf(field(ka)))]
    pub k_a: f64,
    /// The period in seconds.
    #[cfg_attr(feature = "protobuf", protobuf(field(dt)))]
    pub d_t: f64,
}

/// Represents a 2D ellipse space containing translational, rotational, and scaling components.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "Pose2d center;double xSemiAxis;double ySemiAxis"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufEllipse2d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ellipse2d {
    /// The center of the ellipse.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub center: Pose2d,
    /// The x semi-axis.
    #[cfg_attr(feature = "protobuf", protobuf(field(xSemiAxis)))]
    pub x_semi_axis: f64,
    /// The y semi-axis.
    #[cfg_attr(feature = "protobuf", protobuf(field(ySemiAxis)))]
    pub y_semi_axis: f64,
}

/// Exponential profile state.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double position;double velocity"),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExponentialProfileState {
    /// The position at this state
    pub position: f64,
    /// The velocity at this state
    pub velocity: f64,
}

/// Represents a plant defined using state-space notation.
///
/// A plant is a mathematical model of a system's dynamics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearSystem<const S: usize, const I: usize, const O: usize> {
    /// The system matrix A.
    pub a: Matrix<S, S>,
    /// The system matrix B.
    pub b: Matrix<S, I>,
    /// The system matrix C.
    pub c: Matrix<O, S>,
    /// The system matrix D.
    pub d: Matrix<O, I>,
}

#[cfg(feature = "struct")]
impl<const S: usize, const I: usize, const O: usize> StructData for LinearSystem<S, I, O,> {
    fn schema() -> StructSchema {
        StructSchema(format!(
            "{} a;{} b;{} c;{} d",
            <Matrix<S, S>>::struct_type_name(),
            <Matrix<S, I>>::struct_type_name(),
            <Matrix<O, S>>::struct_type_name(),
            <Matrix<O, I>>::struct_type_name(),
        ))
    }

    fn struct_type_name() -> String {
        format!("LinearSystem__{}_{}_{}", S, I, O)
    }

    async fn publish_dependencies(manager: &mut crate::schema::SchemaManager) -> Result<(), PublishSchemaError> {
        manager.publish_struct::<Matrix<S, S>>().await?;
        manager.publish_struct::<Matrix<S, I>>().await?;
        manager.publish_struct::<Matrix<O, S>>().await?;
        manager.publish_struct::<Matrix<O, I>>().await
    }

    fn pack(self, buf: &mut ByteBuffer) {
        self.a.pack(buf);
        self.b.pack(buf);
        self.c.pack(buf);
        self.d.pack(buf);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> where Self: Sized {
        let a = Matrix::unpack(read)?;
        let b = Matrix::unpack(read)?;
        let c = Matrix::unpack(read)?;
        let d = Matrix::unpack(read)?;

        Some(Self { a, b, c, d })
    }
}

#[cfg(feature = "protobuf")]
impl<const S: usize, const I: usize, const O: usize> ProtobufData for LinearSystem<S, I, O> {
    type Proto = ProtobufLinearSystem;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            a: Matrix::from_proto(proto.a.into_option().unwrap_or_default())?,
            b: Matrix::from_proto(proto.b.into_option().unwrap_or_default())?,
            c: Matrix::from_proto(proto.c.into_option().unwrap_or_default())?,
            d: Matrix::from_proto(proto.d.into_option().unwrap_or_default())?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            num_states: S as u32,
            num_inputs: I as u32,
            num_outputs: O as u32,
            a: protobuf::MessageField::some(self.a.into_proto()),
            b: protobuf::MessageField::some(self.a.into_proto()),
            c: protobuf::MessageField::some(self.a.into_proto()),
            d: protobuf::MessageField::some(self.a.into_proto()),
            ..Default::default()
        }
    }
}

/// Represents a RxC matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix<const R: usize, const C: usize> {
    /// The data of the matrix, indexed as `data[row][col]`.
    pub data: [[f64; C]; R],
}

#[cfg(feature = "struct")]
impl<const R: usize, const C: usize> StructData for Matrix<R, C> {
    fn schema() -> StructSchema {
        StructSchema(format!("double data[{}]", R * C))
    }

    fn struct_type_name() -> String {
        format!("Matrix__{}_{}", R, C)
    }

    async fn publish_dependencies(_: &mut crate::schema::SchemaManager) -> Result<(), PublishSchemaError> {
        Ok(())
    }

    fn pack(self, buf: &mut ByteBuffer) {
        self.data.into_iter().flatten().for_each(|value| {
            buf.write_f64(value);
        });
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let mut data = [[0.0; C]; R];
        for value in data.iter_mut().flatten() {
            *value = read.read_f64()?;
        }
        Some(Self { data })
    }
}

#[cfg(feature = "protobuf")]
impl<const R: usize, const C: usize> ProtobufData for Matrix<R, C> {
    type Proto = ProtobufMatrix;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        let mut data = [[0.0; C]; R];
        if proto.data.len() != C * R { return None; };
        proto.data.into_iter()
            .enumerate()
            .for_each(|(i, num)| {
                data[i / C][i % C] = num;
            });
        Some(Self {
            data,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            num_rows: R as u32,
            num_cols: C as u32,
            data: self.data.into_iter().flatten().collect(),
            ..Default::default()
        }
    }
}

/// Kinematics for a mecanum drive.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "Translation2d front_left;Translation2d front_right;Translation2d rear_left;Translation2d rear_right"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufMecanumDriveKinematics))
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MecanumDriveKinematics {
    /// The front-left wheel translation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub front_left: Translation2d,
    /// The front-right wheel translation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub front_right: Translation2d,
    /// The rear-right wheel translation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub rear_left: Translation2d,
    /// The rear-left wheel translation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub rear_right: Translation2d,
}

/// Represents the wheel positions for a mecanum drive drivetrain.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double front_left;double front_right;double rear_left;double rear_right"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufMecanumDriveWheelPositions)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MecanumDriveWheelPositions {
    /// Distance measured by the front-left wheel in meters.
    pub front_left: f64,
    /// Distance measured by the front-right wheel in meters.
    pub front_right: f64,
    /// Distance measured by the rear-left wheel in meters.
    pub rear_left: f64,
    /// Distance measured by the rear-right wheel in meters.
    pub rear_right: f64,
}

/// Represents the wheel speeds for a mecanum drive drivetrain.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double front_left;double front_right;double rear_left;double rear_right"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufMecanumDriveWheelSpeeds)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MecanumDriveWheelSpeeds {
    /// Speed of the front-left wheel in m/s.
    pub front_left: f64,
    /// Speed of the front-right wheel in m/s.
    pub front_right: f64,
    /// Speed of the rear-left wheel in m/s.
    pub rear_left: f64,
    /// Speed of the rear-right wheel in m/s.
    pub rear_right: f64,
}

/// Represents a 2D pose containing translational and rotational elements.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "Translation2d translation;Rotation2d rotation"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufPose2d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2d {
    /// The translation component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub translation: Translation2d,
    /// The rotational component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub rotation: Rotation2d,
}

/// Represents a 3D pose containing translational and rotational elements.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "Translation3d translation;Rotation3d rotation"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufPose3d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose3d {
    /// The translation component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub translation: Translation3d,
    /// The rotational component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub rotation: Rotation3d,
}

/// Represents a quaternion.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double w;double x;double y;double z"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufQuaternion)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// The w component of the quaternion.
    pub w: f64,
    /// The x component of the quaternion.
    pub x: f64,
    /// The y component of the quaternion.
    pub y: f64,
    /// The z component of the quaternion.
    pub z: f64,
}

/// Represents a hermite spline of degree 5.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double xInitial[3];double xFinal[3];double yInitial[3];double yFinal[3]"),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuinticHermiteSpline {
    /// The control vector for the initial point in the x dimension.
    pub x_initial: [f64; 3],
    /// The control vector for the final point in the x dimension.
    pub x_final: [f64; 3],
    /// The control vector for the initial point in the y dimension.
    pub y_initial: [f64; 3],
    /// The control vector for the final point in the y dimension.
    pub y_final: [f64; 3],
}

#[cfg(feature = "protobuf")]
impl ProtobufData for QuinticHermiteSpline {
    type Proto = ProtobufQuinticHermiteSpline;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            x_initial: proto.x_initial.try_into().ok()?,
            x_final: proto.x_final.try_into().ok()?,
            y_initial: proto.y_initial.try_into().ok()?,
            y_final: proto.y_final.try_into().ok()?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            x_initial: self.x_initial.into(),
            x_final: self.x_final.into(),
            y_initial: self.y_initial.into(),
            y_final: self.y_final.into(),
            ..Default::default()
        }
    }
}

/// Represents a 2D rectangular space containing translational, rotational, and scaling components.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "Pose2d center;double xWidth;double yWidth"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufRectangle2d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rectangle2d {
    /// The center of the rectangle.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub center: Pose2d,
    /// The x size component of the rectangle.
    #[cfg_attr(feature = "protobuf", protobuf(field(xWidth)))]
    pub x_width: f64,
    /// The y size component of the rectangle.
    #[cfg_attr(feature = "protobuf", protobuf(field(yWidth)))]
    pub y_width: f64,
}

/// Represents a rotation in a 2D coordinate frame representd by a point on the unit circle.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double value"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufRotation2d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation2d {
    /// The rotation in radians.
    pub value: f64,
}

/// Represents a rotation in a 2D coordinate frame representd by a point on the unit circle.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(type_name = "Rotation3d", schema = "Quaternion q"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufRotation3d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation3d {
    /// The quaternion representation of the rotation.
    #[cfg_attr(feature = "protobuf", protobuf(field(q), nested))]
    #[cfg_attr(feature = "struct", structdata(nested))]
    pub quaternion: Quaternion,
}

/// Feedforward constants that model a simple permanent-magnet DC motor.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double ks;double kv;double ka;double dt"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufSimpleMotorFeedforward)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimpleMotorFeedforward {
    /// The static gain in volts.
    #[cfg_attr(feature = "protobuf", protobuf(field(ks)))]
    pub k_s: f64,
    /// The velocity gain in V/(units/s).
    #[cfg_attr(feature = "protobuf", protobuf(field(kv)))]
    pub k_v: f64,
    /// The acceleration gain in V/(units/s²).
    #[cfg_attr(feature = "protobuf", protobuf(field(ka)))]
    pub k_a: f64,
    /// The period in seconds.
    #[cfg_attr(feature = "protobuf", protobuf(field(dt)))]
    pub d_t: f64,
}

/// Kinematics for a swerve drive with N modules.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwerveDriveKinematics<const N: usize> {
    /// The swerve module locations.
    pub modules: [Translation2d; N],
}

#[cfg(feature = "struct")]
impl<const N: usize> StructData for SwerveDriveKinematics<N> {
    fn schema() -> StructSchema {
        StructSchema(format!("Translation2d modules[{N}]"))
    }

    fn struct_type_name() -> String {
        format!("SwerveDriveKinematics__{}", N)
    }

    async fn publish_dependencies(manager: &mut crate::schema::SchemaManager) -> Result<(), PublishSchemaError> {
        manager.publish_struct::<Translation2d>().await
    }

    fn pack(self, buf: &mut ByteBuffer) {
        Translation2d::pack_iter(self.modules, buf);
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let modules = Translation2d::unpack_array(read)?;
        Some(Self { modules })
    }
}

#[cfg(feature = "protobuf")]
impl<const N: usize> ProtobufData for SwerveDriveKinematics<N> {
    type Proto = ProtobufSwerveDriveKinematics;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            modules: proto.modules.into_iter()
                .map(Translation2d::from_proto)
                .collect::<Option<Vec<_>>>()?
                .try_into()
                .ok()?,
        })
    }
    
    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            modules: self.modules.into_iter()
                .map(|module| module.into_proto())
                .collect(),
            ..Default::default()
        }
    }
}

/// Represents the state of one swerve module.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double distance;Rotation2d angle"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufSwerveModulePosition)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwerveModulePosition {
    /// Distance measured by the wheel of the module in meters.
    pub distance: f64,
    /// Angle of the module.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub angle: Rotation2d,
}

/// Represents the state of one swerve module.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double speed;Rotation2d angle"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufSwerveModuleState)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwerveModuleState {
    /// The speed of the wheel of the module in m/s.
    pub speed: f64,
    /// The angle of the module.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub angle: Rotation2d,
}

/// Represents a time-parameterized trajectory. The trajectory contains of various States that represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    /// The states of the trajectory.
    pub states: Vec<TrajectoryState>,
}

#[cfg(feature = "protobuf")]
impl ProtobufData for Trajectory {
    type Proto = ProtobufTrajectory;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            states: proto.states.into_iter()
                .map(TrajectoryState::from_proto)
                .collect::<Option<_>>()?
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            states: self.states.into_iter()
                .map(|state| state.into_proto())
                .collect(),
            ..Default::default()
        }
    }
}

/// Represents a state at a point in a trajectory.
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufTrajectoryState)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrajectoryState {
    /// The time elapsed since the beginning of the trajectory in seconds.
    pub time: f64,
    /// The speed at that point of the trajector in m/s.
    pub velocity: f64,
    /// The acceleration at that point of the trajectory in m/s².
    pub acceleration: f64,
    /// The pose at that point of the trajectory in meters.
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub pose: Pose2d,
    /// The curvature at that point of the trajectory in rad/m.
    pub curvature: f64,
}

/// Represents a transformation for a Pose2d in the pose's frame.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "Translation2d translation;Rotation2d rotation"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufTransform2d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform2d {
    /// The translation component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub translation: Translation2d,
    /// The rotational component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub rotation: Rotation2d,
}

/// Represents a transformation for a Pose3d in the pose's frame.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "Translation3d translation;Rotation3d rotation"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufTransform3d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform3d {
    /// The translation component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub translation: Translation3d,
    /// The rotational component of the transformation.
    #[cfg_attr(feature = "struct", structdata(nested))]
    #[cfg_attr(feature = "protobuf", protobuf(nested))]
    pub rotation: Rotation3d,
}

/// Represents a translation in 2D space.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double x;double y"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufTranslation2d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Translation2d {
    /// The x component of the translation.
    pub x: f64,
    /// The y component of the translation.
    pub y: f64,
}

/// Represents a translation in 2D space.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double x;double y;double z"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufTranslation3d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Translation3d {
    /// The x component of the translation.
    pub x: f64,
    /// The y component of the translation.
    pub y: f64,
    /// The z component of the translation.
    pub z: f64,
}

/// Trapezoid profile state.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double position;double velocity"),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidProfileState {
    /// The position at this state.
    pub position: f64,
    /// The velocity at this state.
    pub velocity: f64,
}

/// Represents a change in distance along a 2D arc.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double dx;double dy;double dtheta"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufTwist2d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Twist2d {
    /// The linear "dx" component.
    pub dx: f64,
    /// The linear "dy" component.
    pub dy: f64,
    /// The linear "dtheta" component in radians.
    pub dtheta: f64,
}

/// Represents a change in distance along a 3D arc.
#[cfg_attr(feature = "struct",
    derive(StructData),
    structdata(schema = "double dx;double dy;double dz;double rx;double ry;double rz"),
)]
#[cfg_attr(feature = "protobuf",
    derive(ProtobufData),
    protobuf(from(ProtobufTwist3d)),
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Twist3d {
    /// The linear "dx" component.
    pub dx: f64,
    /// The linear "dy" component.
    pub dy: f64,
    /// The linear "dz" component.
    pub dz: f64,
    /// The rotation vector x component in radians.
    pub rx: f64,
    /// The rotation vector y component in radians.
    pub ry: f64,
    /// The rotation vector z component in radians.
    pub rz: f64,
}

/// Represents an N-dimensional Vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector<const N: usize> {
    /// The rows of the vector.
    pub rows: [f64; N],
}

#[cfg(feature = "struct")]
impl<const N: usize> StructData for Vector<N> {
    fn schema() -> StructSchema {
        StructSchema(format!("double data[{N}]"))
    }

    fn struct_type_name() -> String {
        format!("Vector__{}", N)
    }

    async fn publish_dependencies(_: &mut crate::schema::SchemaManager) -> Result<(), PublishSchemaError> {
        Ok(())
    }

    fn pack(self, buf: &mut ByteBuffer) {
        for value in self.rows {
            buf.write_f64(value);
        }
    }

    fn unpack(read: &mut ByteReader) -> Option<Self> {
        let mut rows = [0.0; N];
        for value in rows.iter_mut() {
            *value = read.read_f64()?;
        }
        Some(Self { rows })
    }
}

#[cfg(feature = "protobuf")]
impl<const N: usize> ProtobufData for Vector<N> {
    type Proto = ProtobufVector;

    fn from_proto(proto: Self::Proto) -> Option<Self> {
        Some(Self {
            rows: proto.rows.try_into().ok()?,
        })
    }

    fn into_proto(self) -> Self::Proto {
        Self::Proto {
            rows: self.rows.into(),
            ..Default::default()
        }
    }
}

