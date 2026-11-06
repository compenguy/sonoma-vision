//! NetworkTables `struct` pack/unpack support.

use std::mem::MaybeUninit;

use byte::{ByteBuffer, ByteReader};

use crate::{data::{DataType, NetworkTableData}, schema::{PublishSchemaError, SchemaManager}};

pub mod byte;
pub mod parse;

/// Represents `struct` data that can be sent and received by a `NetworkTables` server.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Struct<T>(pub T);

impl<T: StructData> NetworkTableData for Struct<T> {
    fn data_type() -> DataType {
        DataType::Struct(T::struct_type_name())
    }

    fn from_value(value: rmpv::Value) -> Option<Self> {
        match value {
            rmpv::Value::Binary(bytes) => T::unpack(&mut ByteReader::new(&bytes)).map(Self),
            _ => None,
        }
    }

    fn into_value(self) -> rmpv::Value {
        let mut buf = ByteBuffer::new();
        self.0.pack(&mut buf);
        rmpv::Value::Binary(buf.into())
    }
}

/// Data that can be packed and unpacked from raw bytes, known as a `Struct` in WPILib.
pub trait StructData {
    /// Converts self into data that can be sent and received by a `NetworkTables` server.
    fn into_struct_data(self) -> Struct<Self> where Self: Sized {
        Struct(self)
    }

    /// Returns the type name of this struct.
    ///
    /// This name should be unique.
    fn struct_type_name() -> String;

    /// Returns the struct schema.
    fn schema() -> StructSchema;

    /// Publishes all dependency schemas for this struct.
    fn publish_dependencies(manager: &mut SchemaManager) -> impl Future<Output = Result<(), PublishSchemaError>> + Send;

    /// Puts object contents to `buf`.
    fn pack(self, buf: &mut ByteBuffer);

    /// Deserializes an object from `buf`.
    fn unpack(read: &mut ByteReader) -> Option<Self> where Self: Sized;

    /// Puts an iterator of objects to `buf`.
    fn pack_iter(iter: impl IntoIterator<Item = Self>, buf: &mut ByteBuffer)
    where Self: Sized
    {
        for item in iter {
            item.pack(buf);
        }
    }

    /// Deserializes exactly `size` objects from `buf`.
    fn unpack_vec(read: &mut ByteReader, size: usize) -> Option<Vec<Self>>
    where Self: Sized
    {
        let mut vec = Vec::with_capacity(size);
        for _ in 0..size {
            vec.push(Self::unpack(read)?);
        }
        Some(vec)
    }

    /// Deserializes a fixed-size array of length `S` from `buf`.
    fn unpack_array<const S: usize>(read: &mut ByteReader) -> Option<[Self; S]>
    where Self: Sized
    {
        let mut arr = [const { MaybeUninit::uninit() }; S];
        for i in 0..S {
            match Self::unpack(read) {
                Some(data) => {
                    arr[i].write(data);
                },
                None => {
                    // unpacking failed, we need to manually drop every item that has been unpacked
                    arr.iter_mut().take(i).for_each(|elem| {
                        // SAFETY: it is guaranteed that the array up until this point has been
                        // successfully unpacked and is initialized
                        unsafe { elem.assume_init_drop(); };
                    });
                    return None;
                }
            }
        }
        // SAFETY: it is guaranteed that by this point, every value in `arr` has been initialized
        Some(arr.map(|data| unsafe { data.assume_init() }))
    }
}

impl <T: StructData, const S: usize> NetworkTableData for [Struct<T>; S] {
    fn data_type() -> DataType {
        DataType::StructArray(T::struct_type_name())
    }

    fn from_value(value: rmpv::Value) -> Option<Self> {
        match value {
            rmpv::Value::Binary(bytes) => T::unpack_array(&mut ByteReader::new(&bytes)).map(|array| array.map(Struct)),
            _ => None,
        }
    }

    fn into_value(self) -> rmpv::Value {
        let mut buf = ByteBuffer::new();
        T::pack_iter(self.map(|r#struct| r#struct.0), &mut buf);
        rmpv::Value::Binary(buf.into())
    }
}

/// A struct schema.
///
/// Note that the schema is not checked for validity, make sure it is a valid schema before
/// publishing to the server.
///
/// You can make sure the schema is valid by calling [`parse::parse_schema`] and checking for an error.
pub struct StructSchema(pub String);

impl NetworkTableData for StructSchema {
    fn data_type() -> DataType {
        DataType::StructSchema
    }

    fn from_value(value: rmpv::Value) -> Option<Self> {
        match value {
            rmpv::Value::Binary(value) => String::from_utf8(value).ok().map(Self),
            _ => None,
        }
    }

    fn into_value(self) -> rmpv::Value {
        rmpv::Value::Binary(self.0.into_bytes())
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "math")]
    use crate::math::*;

    use std::sync::Arc;

    use lazy_static::lazy_static;

    use super::*;

    #[test]
    fn test_unpack_arr() {
        #[derive(Debug, Clone, PartialEq)]
        struct MyStruct {
            f: f64,
            u: i32,
        }

        impl StructData for MyStruct {
            fn schema() -> StructSchema {
                StructSchema(String::new())
            }

            fn struct_type_name() -> String {
                String::new()
            }

            async fn publish_dependencies(_: &mut SchemaManager) -> Result<(), PublishSchemaError> {
                Ok(())
            }

            fn pack(self, buf: &mut ByteBuffer) {
                buf.write_f64(self.f);
                buf.write_i32(self.u);
            }

            fn unpack(read: &mut ByteReader) -> Option<Self> {
                Some(Self {
                    f: read.read_f64()?,
                    u: read.read_i32()?,
                })
            }
        }

        let bytes = [
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x20, 0x40, 0xEC, 0xFF, 0xFF, 0xFF, // 8.22, -20
            0x85, 0xEB, 0x51, 0xB8, 0x1E, 0x85, 0xF3, 0xBF, 0x37, 0x00, 0x00, 0x00, // -1.22, 55
        ];

        let buf: ByteBuffer = bytes.into();

        assert_eq!(MyStruct::unpack_array::<1>(&mut buf.read()), Some([MyStruct { f: 8.22, u: -20 }]));
        assert_eq!(MyStruct::unpack_array::<2>(&mut buf.read()), Some([MyStruct { f: 8.22, u: -20 }, MyStruct { f: -1.22, u: 55 }]));
        assert_eq!(MyStruct::unpack_array::<3>(&mut buf.read()), None);

        const DATA_VALUE: i32 = 25;
        lazy_static! {
            static ref DATA: Arc<i32> = Arc::new(DATA_VALUE);
        };

        struct DropTest {
            data: Arc<i32>,
        }

        impl StructData for DropTest {
            fn schema() -> StructSchema {
                StructSchema(String::new())
            }

            fn struct_type_name() -> String {
                String::new()
            }

            async fn publish_dependencies(_: &mut SchemaManager) -> Result<(), PublishSchemaError> {
                Ok(())
            }

            fn pack(self, buf: &mut ByteBuffer) {
                buf.write_i32(*self.data);
            }

            fn unpack(read: &mut ByteReader) -> Option<Self> {
                // don't actually read any data since we're using "global" data
                read.read_i32()?;
                // if successful unpack, increment global Rc reference count
                Some(Self { data: Arc::clone(&DATA) })
            }
        }

        let bytes: [u8; 16] = [0; 16]; // 4 * size_of::<i32> bytes

        let buf: ByteBuffer = bytes.into();

        {
            let arr = DropTest::unpack_array::<4>(&mut buf.read());
            assert!(arr.is_some());
            let arr = arr.unwrap();
            assert!(arr.iter().all(|num| *num.data == DATA_VALUE));
            assert!(arr.iter().all(|num| Arc::ptr_eq(&num.data, &DATA)));
            assert_eq!(Arc::strong_count(&DATA), 5); // 1 + 4 drop tests unpacked
        }

        assert_eq!(Arc::strong_count(&DATA), 1); // arr is dropped

        let arr = DropTest::unpack_array::<5>(&mut buf.read());
        assert!(arr.is_none());
        assert_eq!(Arc::strong_count(&DATA), 1); // failed unpacking, all should be dropped
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_armfeedforward() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x02, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x40,
            0x29, 0x5C, 0x8F, 0xC2, 0xF5, 0xA8, 0x20, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x14, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
        ];

        let r#struct = ArmFeedforward {
            k_s: 2.3,
            k_a: 5.2,
            k_v: 8.33,
            d_t: 1.1,
            k_g: 100.0,
        };
        assert_eq!(ArmFeedforward::struct_type_name(), "ArmFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_chassisspeeds() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x4C, 0x34, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x20, 0x40,
            0xB8, 0x1E, 0x85, 0xEB, 0x51, 0x38, 0x26, 0x40,
        ];

        let r#struct = ChassisSpeeds {
            velocity_x: 20.3,
            velocity_y: 8.3,
            omega: 11.11,
        };
        assert_eq!(ChassisSpeeds::struct_type_name(), "ChassisSpeeds");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_cubichermitespline() {
        let bytes = [
            0x8F, 0xC2, 0xF5, 0x28, 0x5C, 0x8F, 0x0A, 0x40,
            0xB8, 0x1E, 0x85, 0xEB, 0x51, 0x38, 0x22, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0xC0,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x43, 0x87, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x01, 0x40,
            0xAE, 0x47, 0xE1, 0x7A, 0x14, 0xAE, 0xFF, 0xBF,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x0C, 0x56, 0x40,
            0x5C, 0x8F, 0xC2, 0xF5, 0x28, 0x9C, 0x46, 0x40,
        ];

        let r#struct = CubicHermiteSpline {
            y_initial_control_vector: [2.2, -1.98],
            x_initial_control_vector: [3.32, 9.11],
            y_final_control_vector: [88.2, 45.22],
            x_final_control_vector: [-33.0, 744.4],
        };
        assert_eq!(CubicHermiteSpline::struct_type_name(), "CubicHermiteSpline");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_dcmotor() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x28, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x01, 0x40,
            0xD7, 0xA3, 0x70, 0x3D, 0x0A, 0xD7, 0x1D, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x74, 0x40,
        ];

        let r#struct = DCMotor {
            free_speed: 333.0,
            free_current: 7.46,
            nominal_voltage: 12.3,
            stall_current: 2.2,
            stall_torque: 1.0,
        };
        assert_eq!(DCMotor::struct_type_name(), "DCMotor");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_differentialdrivefeedforward() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x24, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x32, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1C, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
        ];

        let r#struct = DifferentialDriveFeedforward {
            velocity_linear: 10.2,
            velocity_angular: 7.2,
            acceleration_angular: 4.0,
            acceleration_linear: 18.44,
        };
        assert_eq!(DifferentialDriveFeedforward::struct_type_name(), "DifferentialDriveFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_differentialdrivekinematics() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x09, 0x40,
        ];

        let r#struct = DifferentialDriveKinematics {
            track_width: 3.2,
        };
        assert_eq!(DifferentialDriveKinematics::struct_type_name(), "DifferentialDriveKinematics");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_differentialdrivewheelpositions() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x14, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x14, 0x40,
        ];

        let r#struct = DifferentialDriveWheelPositions {
            left: 5.2,
            right: 5.11,
        };
        assert_eq!(DifferentialDriveWheelPositions::struct_type_name(), "DifferentialDriveWheelPositions");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_differentialdrivewheelspeeds() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x59, 0x8F, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x7A, 0xC0,
        ];

        let r#struct = DifferentialDriveWheelSpeeds {
            left: 1003.2,
            right: -421.0,
        };
        assert_eq!(DifferentialDriveWheelSpeeds::struct_type_name(), "DifferentialDriveWheelSpeeds");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_differentialdrivewheelvoltages() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x27, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1C, 0xC0,
        ];

        let r#struct = DifferentialDriveWheelVoltages {
            left: 11.8,
            right: -7.2,
        };
        assert_eq!(DifferentialDriveWheelVoltages::struct_type_name(), "DifferentialDriveWheelVoltages");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_elevatorfeedforward() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1C, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xF4, 0x3F,
            0x52, 0xB8, 0x1E, 0x85, 0xEB, 0x51, 0x19, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1C, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xC9, 0x3F,
        ];

        let r#struct = ElevatorFeedforward {
            k_s: 7.2,
            k_a: 7.1,
            k_v: 6.33,
            d_t: 0.2,
            k_g: 1.3,
        };
        assert_eq!(ElevatorFeedforward::struct_type_name(), "ElevatorFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_ellipse2d() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x20, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x01, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x86, 0x54, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x0B, 0x40,
        ];

        let r#struct = Ellipse2d {
            center: Pose2d { translation: Translation2d { x: 8.3, y: 2.2 }, rotation: Rotation2d { value: 82.1 } },
            y_semi_axis: 3.4,
            x_semi_axis: 8.2,
        };
        assert_eq!(Ellipse2d::struct_type_name(), "Ellipse2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_exponentialprofilestate() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x06, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xE1, 0xBF,
        ];

        let r#struct = ExponentialProfileState {
            velocity: -0.55,
            position: 2.8,
        };
        assert_eq!(ExponentialProfileState::struct_type_name(), "ExponentialProfileState");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_linearsystem() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x02, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x12, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x18, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x1A, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xF3, 0x3F,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x09, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x43, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x48, 0xC0,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x43, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x5B, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xC6, 0x5B, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x72, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x28, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x4D, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x15, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xC6, 0x50, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x89, 0x7D, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x8A, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xE4, 0x81, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x0A, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xC0,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x09, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x3A, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x16, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xAC, 0x53, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1C, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x46, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xB6, 0x71, 0xC0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0xC0,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x20, 0x40,
        ];

        let r#struct = LinearSystem::<3, 5, 2> {
            a: Matrix { data: [[2.3, 1.1, 4.6], [2.1, 6.2, 6.7], [1.2, 3.2, 38.8]] },
            b: Matrix { data: [[-48.2, 38.2, -111.0, 8.2, 111.1], [302.0, 12.3, 58.2, 1.1, 5.3], [67.1, 472.6, 837.0, 72.0, 44.0]] },
            c: Matrix { data: [[2.1, 572.6, 3.3], [8.0, -11.0, 3.2]] },
            d: Matrix { data: [[26.1, 5.6, 78.7, 7.1, 44.5], [9.0, 11.0, -283.4, -88.0, 8.22]] },
        };
        assert_eq!(LinearSystem::<3, 5, 2>::struct_type_name(), "LinearSystem__3_5_2");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_matrix() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0xC0,
            0xAE, 0x47, 0xE1, 0x7A, 0x14, 0xAE, 0x16, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x19, 0x48, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x14, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x40,
        ];

        let r#struct = Matrix::<2, 3> {
            data: [[-8.2, 5.67, 48.2], [4.2, 5.11, 9.5]],
        };
        assert_eq!(Matrix::<2, 3>::struct_type_name(), "Matrix__2_3");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_mecanumdrivekinematics() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0xC0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0xC0,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0xC0,
        ];

        let r#struct = MecanumDriveKinematics {
            front_right: Translation2d { x: 4.2, y: 4.0 },
            rear_right: Translation2d { x: 4.2, y: -8.2 },
            rear_left: Translation2d { x: -2.1, y: -8.2 },
            front_left: Translation2d { x: -2.1, y: 4.0 },
        };
        assert_eq!(MecanumDriveKinematics::struct_type_name(), "MecanumDriveKinematics");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_mecanumdrivewheelpositions() {
        let bytes = [
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0xF3, 0x68, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x36, 0xC0,
        ];

        let r#struct = MecanumDriveWheelPositions {
            front_right: 199.6,
            rear_right: -22.2,
            rear_left: 9.0,
            front_left: 8.2,
        };
        assert_eq!(MecanumDriveWheelPositions::struct_type_name(), "MecanumDriveWheelPositions");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_mecanumdrivewheelspeeds() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x18, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x8B, 0x40,
            0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xA1, 0x55, 0xC0,
        ];

        let r#struct = MecanumDriveWheelSpeeds {
            front_right: 4.5,
            rear_right: -86.52,
            rear_left: 881.0,
            front_left: 6.2,
        };
        assert_eq!(MecanumDriveWheelSpeeds::struct_type_name(), "MecanumDriveWheelSpeeds");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_pose2d() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x3C, 0x91, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xEC, 0x81, 0xC0,
            0xC3, 0xF5, 0x28, 0x5C, 0x8F, 0xC2, 0x1D, 0x40,
        ];

        let r#struct = Pose2d {
            translation: Translation2d { x: -1103.2, y: -573.55 },
            rotation: Rotation2d { value: 7.44 },
        };
        assert_eq!(Pose2d::struct_type_name(), "Pose2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_pose3d() {
        let bytes = [
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0x0C, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1B, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x11, 0x40,
            0x54, 0xF7, 0x96, 0x9A, 0x56, 0xC2, 0xD0, 0x3F,
            0xC6, 0x49, 0xC9, 0x78, 0x73, 0x58, 0xB6, 0x3F,
            0xC6, 0x49, 0xC9, 0x78, 0x73, 0x58, 0xA6, 0xBF,
            0x71, 0xC5, 0x14, 0xC6, 0x9E, 0xB9, 0xEE, 0x3F,
        ];

        let r#struct = Pose3d {
            translation: Translation3d { x: 3.56, y: 6.8, z: 4.4 },
            rotation: Rotation3d { quaternion: Quaternion { w: 0.2618614682831908, x: 0.08728715609439694, y: -0.04364357804719847, z: 0.9601587170383664 } },
        };
        assert_eq!(Pose3d::struct_type_name(), "Pose3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_quaternion() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x23, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x16, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0xA6, 0x4B, 0x40,
        ];

        let r#struct = Quaternion {
            w: 9.9,
            x: 2.1,
            y: 5.6,
            z: 55.3,
        };
        assert_eq!(Quaternion::struct_type_name(), "Quaternion");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_quintichermitespline() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x15, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x10, 0x40,
            0x85, 0xEB, 0x51, 0xB8, 0x1E, 0x05, 0x22, 0x40,
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0x84, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x0C, 0x46, 0x40,
            0xA4, 0x70, 0x3D, 0x0A, 0xD7, 0xA3, 0x1A, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x10, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x1F, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x26, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x12, 0x40,
        ];

        let r#struct = QuinticHermiteSpline {
            y_initial: [6.66, 4.2, 7.8],
            x_final: [0.01, 8.2, 44.1],
            y_final: [11.3, 8.5, 4.6],
            x_initial: [5.3, 4.11, 9.01],
        };
        assert_eq!(QuinticHermiteSpline::struct_type_name(), "QuinticHermiteSpline");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_rectangle2d() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x21, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x2C, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x20, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x23, 0x40,
        ];

        let r#struct = Rectangle2d {
            x_width: 8.2,
            center: Pose2d { translation: Translation2d { x: 8.9, y: 14.4 }, rotation: Rotation2d { value: 0.0 } },
            y_width: 9.6,
        };
        assert_eq!(Rectangle2d::struct_type_name(), "Rectangle2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_rotation2d() {
        let bytes = [
            0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xE1, 0x20, 0x40,
        ];

        let r#struct = Rotation2d {
            value: 8.44,
        };
        assert_eq!(Rotation2d::struct_type_name(), "Rotation2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_rotation3d() {
        let bytes = [
            0xF7, 0xC6, 0x57, 0x88, 0x7B, 0xA7, 0xC8, 0x3F,
            0xFC, 0x93, 0xE4, 0x36, 0xE8, 0x3F, 0xDC, 0x3F,
            0x50, 0x8C, 0x8F, 0xF3, 0xC7, 0x57, 0xE6, 0x3F,
            0xCA, 0x58, 0xBC, 0xED, 0x24, 0xF3, 0xE0, 0x3F,
        ];

        let r#struct = Rotation3d {
            quaternion: Quaternion { w: 0.19261116177909063, x: 0.44140057907708274, y: 0.6982154614492035, z: 0.5296806948924992 },
        };
        assert_eq!(Rotation3d::struct_type_name(), "Rotation3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_simplemotorfeedforward() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xF1, 0x3F,
            0x1F, 0x85, 0xEB, 0x51, 0xB8, 0x1E, 0xE5, 0x3F,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0x4C, 0x32, 0x40,
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0xD4, 0x3F,
        ];

        let r#struct = SimpleMotorFeedforward {
            k_s: 1.1,
            d_t: 0.32,
            k_v: 0.66,
            k_a: 18.3,
        };
        assert_eq!(SimpleMotorFeedforward::struct_type_name(), "SimpleMotorFeedforward");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_swervedrivekinematics() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x11, 0x40,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1A, 0x40,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC0,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x1A, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x11, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x23, 0xC0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC0,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x23, 0xC0,
        ];

        let r#struct = SwerveDriveKinematics::<4> {
            modules: [Translation2d { x: 4.3, y: 6.6 }, Translation2d { x: -5.0, y: 6.6 }, Translation2d { x: 4.3, y: -9.9 }, Translation2d { x: -5.0, y: -9.9 }],
        };
        assert_eq!(SwerveDriveKinematics::<4>::struct_type_name(), "SwerveDriveKinematics__4");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_swervemoduleposition() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x00, 0x40,
            0x48, 0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xE9, 0x3F,
        ];

        let r#struct = SwerveModulePosition {
            angle: Rotation2d { value: 0.79 },
            distance: 2.1,
        };
        assert_eq!(SwerveModulePosition::struct_type_name(), "SwerveModulePosition");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_swervemodulestate() {
        let bytes = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x40,
            0x6F, 0x12, 0x83, 0xC0, 0xCA, 0x21, 0x09, 0x40,
        ];

        let r#struct = SwerveModuleState {
            speed: 5.5,
            #[allow(clippy::approx_constant)]
            angle: Rotation2d { value: 3.1415 },
        };
        assert_eq!(SwerveModuleState::struct_type_name(), "SwerveModuleState");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_transform2d() {
        let bytes = [
            0xC3, 0xF5, 0x28, 0x5C, 0x8F, 0xC2, 0x09, 0x40,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x19, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x22, 0x40,
        ];

        let r#struct = Transform2d {
            translation: Translation2d { x: 3.22, y: 6.45 },
            rotation: Rotation2d { value: 9.22 },
        };
        assert_eq!(Transform2d::struct_type_name(), "Transform2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_transform3d() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x03, 0xC0,
            0x7B, 0x14, 0xAE, 0x47, 0xE1, 0x7A, 0x17, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x22, 0x40,
            0xC3, 0x48, 0x2E, 0x27, 0xAC, 0xB7, 0xE5, 0x3F,
            0x65, 0xE9, 0x61, 0x02, 0x39, 0xD4, 0xD3, 0x3F,
            0xB7, 0x8B, 0x0A, 0x68, 0xDA, 0x44, 0xE5, 0xBF,
            0x15, 0x19, 0xC8, 0x94, 0xF2, 0xC5, 0xA4, 0x3F,
        ];

        let r#struct = Transform3d {
            translation: Translation3d { x: -2.4, y: 5.87, z: 9.1 },
            rotation: Rotation3d { quaternion: Quaternion { w: 0.6786709561586338, x: 0.3098280452028546, y: -0.6646549255423143, z: 0.04057272020513573 } },
        };
        assert_eq!(Transform3d::struct_type_name(), "Transform3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_translation2d() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x20, 0x40,
            0x0A, 0xD7, 0xA3, 0x70, 0x3D, 0x0A, 0xF7, 0x3F,
        ];

        let r#struct = Translation2d {
            x: 8.1,
            y: 1.44,
        };
        assert_eq!(Translation2d::struct_type_name(), "Translation2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_translation3d() {
        let bytes = [
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x22, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x1B, 0x40,
            0xF6, 0x28, 0x5C, 0x8F, 0xC2, 0xF5, 0x10, 0x40,
        ];

        let r#struct = Translation3d {
            x: 9.1,
            y: 6.9,
            z: 4.24,
        };
        assert_eq!(Translation3d::struct_type_name(), "Translation3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_trapezoidprofilestate() {
        let bytes = [
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x56, 0x40,
            0xAE, 0x47, 0xE1, 0x7A, 0x14, 0x8E, 0x41, 0x40,
        ];

        let r#struct = TrapezoidProfileState {
            velocity: 35.11,
            position: 91.2,
        };
        assert_eq!(TrapezoidProfileState::struct_type_name(), "TrapezoidProfileState");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_twist2d() {
        let bytes = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,
            0x48, 0xE1, 0x7A, 0x14, 0xAE, 0x47, 0xF5, 0x3F,
            0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xAC, 0x52, 0x40,
        ];

        let r#struct = Twist2d {
            dx: 4.0,
            dy: 1.33,
            dtheta: 74.7,
        };
        assert_eq!(Twist2d::struct_type_name(), "Twist2d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_twist3d() {
        let bytes = [
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xD9, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x12, 0x40,
            0x71, 0x3D, 0x0A, 0xD7, 0xA3, 0x70, 0x20, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0x15, 0x40,
            0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xE9, 0x3F,
            0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0xF6, 0x3F,
        ];

        let r#struct = Twist3d {
            ry: 0.8,
            rz: 1.4,
            dx: 0.4,
            dy: 4.6,
            dz: 8.22,
            rx: 5.4,
        };
        assert_eq!(Twist3d::struct_type_name(), "Twist3d");
        test_struct(r#struct, &bytes);
    }

    #[test]
    #[cfg(feature = "math")]
    fn test_vector() {
        let bytes = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x40,
            0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xE3, 0xBF,
            0xF6, 0x28, 0x5C, 0x8F, 0xC2, 0xF5, 0x1F, 0x40,
        ];

        let r#struct = Vector::<3> {
            rows: [4.5, -0.6, 7.99],
        };
        assert_eq!(Vector::<3>::struct_type_name(), "Vector__3");
        test_struct(r#struct, &bytes);
    }

    fn test_struct<S>(s: S, matches: &[u8])
    where S: StructData + std::clone::Clone + std::fmt::Debug + std::cmp::PartialEq
    {
        let mut buf = ByteBuffer::new();
        s.clone().pack(&mut buf);

        let bytes: Vec<u8> = buf.into();
        assert_eq!(&bytes, matches);

        assert_eq!(S::unpack(&mut ByteReader::new(matches)), Some(s));
    }
}

