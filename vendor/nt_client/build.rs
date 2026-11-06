#[cfg(feature = "protobuf")]
fn main() {
    protobuf_codegen::Codegen::new()
        .protoc()
        .protoc_path(&protoc_bin_vendored::protoc_bin_path().unwrap())
        .include("src/protobuf")
        .input("src/protobuf/controller.proto")
        .input("src/protobuf/geometry2d.proto")
        .input("src/protobuf/geometry3d.proto")
        .input("src/protobuf/kinematics.proto")
        .input("src/protobuf/plant.proto")
        .input("src/protobuf/spline.proto")
        .input("src/protobuf/system.proto")
        .input("src/protobuf/trajectory.proto")
        .input("src/protobuf/wpimath.proto")
        .cargo_out_dir("protobuf")
        .run_from_script();
}

#[cfg(not(feature = "protobuf"))]
fn main() { }

