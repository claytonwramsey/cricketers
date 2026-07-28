use cmake::Config;
use std::{
    env,
    path::{Path, PathBuf},
    process::Command,
};

fn vendor_dir(manifest_dir: &Path, name: &str) -> PathBuf {
    manifest_dir.join("vendor").join(name)
}

/// Configures, builds, and installs a vendored CMake dependency into the shared `prefix`,
/// with its own build tree under `build_root/<name>` so concurrent dependencies don't collide.
fn build_dep(
    manifest_dir: &Path,
    build_root: &Path,
    prefix: &Path,
    name: &str,
    defines: &[(&str, &str)],
) {
    let src = vendor_dir(manifest_dir, name);
    assert!(
        src.join("CMakeLists.txt").exists(),
        "{} has no CMakeLists.txt -- run `git submodule update --init --recursive`",
        src.display()
    );

    let mut cfg = Config::new(&src);
    cfg.out_dir(build_root.join(name))
        .define("CMAKE_INSTALL_PREFIX", prefix)
        .define("CMAKE_PREFIX_PATH", prefix)
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON")
        .always_configure(false);
    for (k, v) in defines {
        cfg.define(k, v);
    }
    cfg.build();
}

/// cricket's CMakeLists.txt unconditionally downloads CPM.cmake from GitHub at configure
/// time. Since we vendor CPM.cmake ourselves (as a submodule), this patch swaps that
/// download for an `include()` of our copy, gated by a CRICKET_SYS_CPM_PATH define.
/// Applied idempotently, mirroring how cricket's own CMakeLists.txt patches CppADCodeGen.
fn apply_cricket_patch(cricket_src: &Path, patch: &Path) {
    let already_applied = Command::new("git")
        .args(["apply", "-p0", "--reverse", "--check"])
        .arg(patch)
        .current_dir(cricket_src)
        .status()
        .expect("failed to run git")
        .success();

    if !already_applied {
        let status = Command::new("git")
            .args(["apply", "-p0"])
            .arg(patch)
            .current_dir(cricket_src)
            .status()
            .expect("failed to run git");
        assert!(status.success(), "failed to apply {}", patch.display());
    }
}

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let build_root = out_dir.join("build");
    let prefix = out_dir.join("prefix");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=patches");

    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "tinyxml2",
        &[
            ("tinyxml2_BUILD_TESTING", "OFF"),
            ("BUILD_SHARED_LIBS", "OFF"),
        ],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "console_bridge",
        &[("BUILD_TESTING", "OFF"), ("BUILD_SHARED_LIBS", "OFF")],
    );
    build_dep(&manifest_dir, &build_root, &prefix, "urdfdom_headers", &[]);
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "urdfdom",
        &[("BUILD_TESTING", "OFF"), ("BUILD_SHARED_LIBS", "OFF")],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "eigen",
        &[("BUILD_TESTING", "OFF"), ("EIGEN_BUILD_DOC", "OFF")],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "fmt",
        &[
            ("BUILD_SHARED_LIBS", "OFF"),
            ("FMT_TEST", "OFF"),
            ("FMT_DOC", "OFF"),
            // Avoid the "d" debug-postfix so the installed archive is always libfmt.a,
            // regardless of CMAKE_BUILD_TYPE, matching the fixed rustc-link-lib name below.
            ("FMT_DEBUG_POSTFIX", ""),
        ],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "nlohmann_json",
        &[("JSON_BuildTests", "OFF"), ("JSON_Install", "ON")],
    );
    build_dep(&manifest_dir, &build_root, &prefix, "cppad", &[]);
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "cgal",
        &[
            // cricket only instantiates CGAL's inexact-constructions kernel, so GMP/MPFR
            // (autotools-only, no clean CMake path) can be skipped entirely.
            ("CGAL_DISABLE_GMP", "ON"),
            ("CGAL_WITH_examples", "OFF"),
            ("CGAL_WITH_demos", "OFF"),
            ("CGAL_WITH_tests", "OFF"),
            ("CGAL_WITH_benchmarks", "OFF"),
        ],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "assimp",
        &[
            ("BUILD_SHARED_LIBS", "OFF"),
            ("ASSIMP_BUILD_ZLIB", "ON"),
            ("ASSIMP_BUILD_TESTS", "OFF"),
            ("ASSIMP_BUILD_ASSIMP_TOOLS", "OFF"),
            ("ASSIMP_BUILD_SAMPLES", "OFF"),
            ("ASSIMP_NO_EXPORT", "ON"),
            ("ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT", "OFF"),
            ("ASSIMP_BUILD_STL_IMPORTER", "ON"),
            ("ASSIMP_BUILD_OBJ_IMPORTER", "ON"),
            ("ASSIMP_BUILD_COLLADA_IMPORTER", "ON"),
            ("ASSIMP_BUILD_PLY_IMPORTER", "ON"),
            ("ASSIMP_BUILD_DRACO", "OFF"),
            ("ASSIMP_HUNTER_ENABLED", "OFF"),
        ],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "boost",
        &[
            ("BUILD_SHARED_LIBS", "OFF"),
            ("BUILD_TESTING", "OFF"),
            // math is a header-only Boost library that Pinocchio depends on but never
            // links, so it must be requested explicitly or its headers never get
            // installed and the compiler silently falls back to (a possibly ancient)
            // system Boost.
            ("BOOST_INCLUDE_LIBRARIES", "filesystem;serialization;math"),
        ],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "coal",
        &[
            ("BUILD_SHARED_LIBS", "OFF"),
            ("BUILD_TESTING", "OFF"),
            ("BUILD_PYTHON_INTERFACE", "OFF"),
            ("COAL_HAS_QHULL", "OFF"),
            ("INSTALL_DOCUMENTATION", "OFF"),
        ],
    );
    build_dep(
        &manifest_dir,
        &build_root,
        &prefix,
        "pinocchio",
        &[
            ("BUILD_SHARED_LIBS", "OFF"),
            ("BUILD_TESTING", "OFF"),
            ("BUILD_PYTHON_INTERFACE", "OFF"),
            ("BUILD_EXAMPLES", "OFF"),
            ("BUILD_BENCHMARK", "OFF"),
            ("BUILD_UTILS", "OFF"),
            ("BUILD_WITH_URDF_SUPPORT", "ON"),
            ("BUILD_WITH_COLLISION_SUPPORT", "ON"),
            ("BUILD_WITH_AUTODIFF_SUPPORT", "OFF"),
            ("BUILD_WITH_CASADI_SUPPORT", "OFF"),
            ("BUILD_WITH_OPENMP_SUPPORT", "OFF"),
            ("BUILD_WITH_EXTRA_SUPPORT", "OFF"),
            ("INSTALL_DOCUMENTATION", "OFF"),
        ],
    );

    let cricket_src = vendor_dir(&manifest_dir, "cricket");
    apply_cricket_patch(
        &cricket_src,
        &manifest_dir.join("patches/cricket-vendor-cpm.patch"),
    );

    // cricket's own CMakeLists.txt fetches cxxopts and CppADCodeGen via CPM; pointing
    // CPM_<Name>_SOURCE at our submodules makes CPM use them directly with no network
    // fetch, regardless of DOWNLOAD_ONLY mode.
    let cpm_path = manifest_dir.join("vendor/cpm-cmake/cmake/CPM.cmake");
    let cxxopts_src = vendor_dir(&manifest_dir, "cxxopts");
    let cppadcodegen_src = vendor_dir(&manifest_dir, "cppadcodegen");

    // CppAD has no CMake package config, only a .pc file, matching cricket's own
    // find_package/pkg_check_modules fallback.
    let pkg_config_path = format!(
        "{}:{}",
        prefix.join("lib/pkgconfig").display(),
        prefix.join("share/pkgconfig").display(),
    );

    Config::new(&cricket_src)
        .out_dir(build_root.join("cricket"))
        .define("CMAKE_INSTALL_PREFIX", &prefix)
        .define("CMAKE_PREFIX_PATH", &prefix)
        .define("CRICKET_SYS_CPM_PATH", &cpm_path)
        .define("CRICKET_BUILD_JIT", "OFF")
        .define("CRICKET_BUILD_PYTHON", "OFF")
        .define("CPM_cxxopts_SOURCE", &cxxopts_src)
        .define("CPM_CppADCodeGen_SOURCE", &cppadcodegen_src)
        .always_configure(false)
        .env("PKG_CONFIG_PATH", &pkg_config_path)
        .build();

    let lib_dir = prefix.join("lib");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());

    // Static libs.
    println!("cargo:rustc-link-lib=static=cricket");
    println!("cargo:rustc-link-lib=static=fmt");
    println!("cargo:rustc-link-lib=static=boost_filesystem");
    println!("cargo:rustc-link-lib=static=boost_serialization");
    // Shared libs (urdfdom, coal, cppad, and pinocchio all hardcode SHARED regardless of
    // BUILD_SHARED_LIBS); their own transitive shared-lib deps (console_bridge, tinyxml2,
    // assimp) are baked into their DT_NEEDED entries and don't need to be listed here.
    println!("cargo:rustc-link-lib=dylib=pinocchio_default");
    println!("cargo:rustc-link-lib=dylib=pinocchio_parsers");
    println!("cargo:rustc-link-lib=dylib=pinocchio_collision");
    println!("cargo:rustc-link-lib=dylib=coal");
    println!("cargo:rustc-link-lib=dylib=urdfdom_model");
    println!("cargo:rustc-link-lib=dylib=urdfdom_world");
    println!("cargo:rustc-link-lib=dylib=urdfdom_sensor");
    println!("cargo:rustc-link-lib=dylib=cppad_lib");

    println!("cargo:root={}", prefix.display());
}
