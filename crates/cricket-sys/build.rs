use cmake::Config;
use std::{
    env,
    hash::{DefaultHasher, Hash, Hasher},
    path::{Path, PathBuf},
    process::Command,
};

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let build_root = out_dir.join("build");
    let prefix = out_dir.join("prefix");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=patches");
    println!("cargo:rerun-if-changed=csrc");

    let stamp_path = prefix.join(".build-stamp");
    let fingerprint = build_fingerprint(&manifest_dir).to_string();
    if std::fs::read_to_string(&stamp_path)
        .ok()
        .is_none_or(|cached_fingerprint| cached_fingerprint != fingerprint)
        || !prefix.join("lib/libcricket.a").exists()
    {
        build_cricket(
            &manifest_dir,
            &build_root,
            &prefix,
            &stamp_path,
            &fingerprint,
        );
    }

    compile_shim(&manifest_dir, &prefix);
    emit_link_directives(&prefix);
}

/// Build the cricket library and then update the stamp with the new fingerprint.
fn build_cricket(
    manifest_dir: &Path,
    build_root: &Path,
    prefix: &Path,
    stamp_path: &Path,
    fingerprint: &str,
) {
    build_dep(
        manifest_dir,
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
            // must make a shared lib to satisfy dependents.
            ("BUILD_SHARED_LIBS", "ON"),
            ("BUILD_TESTING", "OFF"),
            // filesystem and serialization are the only libraries pinocchio/coal actually
            // link against; everything else here is a header-only library one of them
            // #includes directly with no trace of it in any find_package/CMakeLists.txt,
            // discovered by grepping pinocchio/coal/cricket for every boost/<x> include.
            (
                "BOOST_INCLUDE_LIBRARIES",
                "filesystem;serialization;algorithm;asio;bind;config;container;core;detail;\
                 foreach;function;fusion;graph;integer;iostreams;iterator;logic;math;mpl;\
                 multiprecision;optional;preprocessor;property_tree;smart_ptr;type_traits;\
                 variant",
            ),
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
        .configure_arg("--no-warn-unused-cli")
        .env("PKG_CONFIG_PATH", &pkg_config_path)
        .build();

    std::fs::write(&stamp_path, &fingerprint).unwrap();
}

fn vendor_dir(manifest_dir: &Path, name: &str) -> PathBuf {
    manifest_dir.join("vendor").join(name)
}

/// Hashes build.rs, the patches directory, and (via `git submodule status --recursive`) the
/// exact pinned commit of every vendored dependency, including nested submodules like
/// boost's own libs/* or pinocchio/coal's jrl-cmakemodules. `.gitmodules` only records each
/// submodule's URL, not its pinned commit, so this is the only way to detect "someone bumped
/// a vendored dependency" at all.
fn build_fingerprint(manifest_dir: &Path) -> u64 {
    let mut hasher = DefaultHasher::new();
    std::fs::read(manifest_dir.join("build.rs"))
        .unwrap()
        .hash(&mut hasher);

    let mut patch_files: Vec<_> = std::fs::read_dir(manifest_dir.join("patches"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    patch_files.sort();
    for path in patch_files {
        std::fs::read(path).unwrap().hash(&mut hasher);
    }

    let submodule_status = Command::new("git")
        .args(["submodule", "status", "--recursive"])
        .current_dir(manifest_dir)
        .output()
        .expect("failed to run git");
    assert!(
        submodule_status.status.success(),
        "git submodule status failed"
    );
    submodule_status.stdout.hash(&mut hasher);

    hasher.finish()
}

/// Compiles the `cxx` bridge (src/ffi.rs) together with its hand-written C++ implementation
/// (csrc/shim.{h,cc}) around cricket's C++ API. Must run before `emit_link_directives`:
/// `cxx_build::bridge(..).compile` emits its own `cargo:rustc-link-lib` for the shim archive
/// immediately, and it has to appear before `-lcricket` on the link line since the shim
/// references cricket's symbols, not the other way around.
fn compile_shim(manifest_dir: &Path, prefix: &Path) {
    let include = prefix.join("include");
    // Deliberately relative (cxx_build derives the generated header's install path,
    // "cricket-sys/src/ffi.rs.h", from this path) -- build scripts always run with `manifest_dir`
    // as their working directory, so this resolves the same as `manifest_dir.join("src/ffi.rs")`.
    cxx_build::bridge("src/ffi.rs")
        .file(manifest_dir.join("csrc/shim.cc"))
        .std("c++17")
        .include(manifest_dir.join("csrc"))
        .include(&include)
        .include(include.join("eigen3"))
        .include(include.join("pinocchio/deprecated"))
        .include(include.join("urdfdom"))
        .include(include.join("urdfdom_headers"))
        // Unlike the old shim, this one instantiates `std::unique_ptr<cricket::RobotInfo>`'s
        // destructor (via `robot_info_new`), which pulls in the full definition of
        // `pinocchio::Model` -- including its `boost::variant` of ~25 joint types, which
        // overflows `boost::mpl::list`'s default 20-type limit. These match pinocchio's own
        // `INTERFACE_COMPILE_DEFINITIONS` (see
        // `prefix/lib/cmake/pinocchio/pinocchioTargets.cmake`); we have to repeat them by
        // hand since we consume pinocchio via raw `-I` paths, not a CMake target, so
        // nothing propagates these to us automatically.
        .define("BOOST_MPL_LIMIT_LIST_SIZE", "30")
        .define("BOOST_MPL_LIMIT_VECTOR_SIZE", "30")
        .define("BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS", None)
        .define("BOOST_FUSION_INVOKE_MAX_ARITY", "12")
        .define("PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION", None)
        .define("PINOCCHIO_WITH_COLLISION", None)
        .define("PINOCCHIO_WITH_HPP_FCL", None)
        .define("PINOCCHIO_WITH_URDFDOM", None)
        .define("PINOCCHIO_URDFDOM_HEADERS_MAJOR_VERSION", "3")
        .define("PINOCCHIO_URDFDOM_HEADERS_MINOR_VERSION", "0")
        .define("PINOCCHIO_URDFDOM_HEADERS_PATCH_VERSION", "0")
        .compile("cricket_shim");

    println!("cargo:rerun-if-changed=src/ffi.rs");
}

fn emit_link_directives(prefix: &Path) {
    let lib_dir = prefix.join("lib");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
    // rustc defaults to lld on this toolchain, which rejects the .debug_gdb_scripts section
    // GCC emits into our vendored static libs in debug builds ("string is not null terminated")
    // -- ld.bfd and gold both accept it fine, and a later -fuse-ld= wins over the earlier
    // toolchain-default one already on the link line.
    println!("cargo:rustc-link-arg=-fuse-ld=bfd");

    // Static libs.
    println!("cargo:rustc-link-lib=static=cricket");
    println!("cargo:rustc-link-lib=static=fmt");
    // Shared libs.
    // Libraries depending on Boost hardcode a shared lib requirement, so we must require boost as a
    // shared lib.
    println!("cargo:rustc-link-lib=dylib=boost_filesystem");
    println!("cargo:rustc-link-lib=dylib=boost_serialization");
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
        .always_configure(false)
        // the cmake crate always passes CMAKE_ASM_COMPILER/CMAKE_ASM_FLAGS even though none
        // of our vendored dependencies use ASM as a project language.
        .configure_arg("--no-warn-unused-cli");
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
