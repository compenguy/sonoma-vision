//! Compile napa-vision's C core (vendor/napa-vision) for `detector::Backend::Napa`, with the flags
//! napa-vision's own builds use.

fn main() {
    let dir = "vendor/napa-vision";
    println!("cargo:rerun-if-changed={dir}");
    cc::Build::new()
        .file(format!("{dir}/napa_core.c"))
        .include(dir)
        .std("c99")
        .opt_level(3)
        .define("NDEBUG", None)
        .flag_if_supported("-fno-strict-overflow")
        .flag_if_supported("-fno-wrapv")
        .warnings(false)
        .compile("napa_core");
}
