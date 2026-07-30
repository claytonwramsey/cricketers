//! Raw `cxx` bridge to cricket's `RobotInfo` and free `trace_*`/`generate_robot_source`
//! functions (`cricket/robot_info.hh`, `cricket/codegen.hh`). This module is the entire
//! low-level surface of `cricket-sys`: every function here is implemented in
//! `csrc/shim.{h,cc}` as a free function taking an explicit `&RobotInfo` (or, for the one
//! genuine mutator, `Pin<&mut RobotInfo>`), never as a C++ member-call, so it stays a thin,
//! literal mirror of the upstream API rather than a redesign of it.
//!
//! Functions that can throw on the C++ side (anything touching file I/O, URDF/JSON parsing, or
//! inja template rendering) return `Result<_, cxx::Exception>`: `cxx` wraps the underlying call
//! in a `try`/`catch` automatically, so `csrc/shim.cc` never has to do that bookkeeping itself.

#[cxx::bridge(namespace = "cricket_ffi")]
mod bridge {
    /// Optional Cartesian bounds for FreeFlyer / Planar joints, matching `cricket::Bounds`.
    #[derive(Clone, Copy, Debug, Default)]
    struct Bounds {
        lower: [f64; 3],
        upper: [f64; 3],
    }

    /// Mirrors `cricket::SphereInfo`, with the `pinocchio::SE3 relative` field decomposed into
    /// a translation and a row-major 3x3 rotation matrix (`rotation[3*r + c]`), since `SE3`
    /// itself can't cross the FFI boundary.
    #[derive(Clone, Copy, Debug, Default)]
    struct SphereInfo {
        geom_index: usize,
        radius: f32,
        parent_joint: usize,
        parent_frame: usize,
        translation: [f64; 3],
        rotation: [f64; 9],
    }

    /// One entry of `cricket::RobotInfo::bounding_spheres` (a `map<size_t, SphereInfo>`).
    #[derive(Clone, Copy, Debug, Default)]
    struct BoundingSphere {
        frame_index: usize,
        sphere: SphereInfo,
    }

    /// One entry of `cricket::RobotInfo::allowed_link_pairs` (a `set<pair<size_t, size_t>>`).
    #[derive(Clone, Copy, Debug, Default)]
    struct LinkPair {
        first: usize,
        second: usize,
    }

    /// One entry of `cricket::RobotInfo::per_link_spheres` (a `vector<vector<size_t>>`),
    /// flattened one level: `link_index` mirrors the outer index.
    #[derive(Clone, Debug, Default)]
    struct LinkSpheres {
        link_index: usize,
        sphere_indices: Vec<usize>,
    }

    /// Mirrors `cricket::Traced`, the result of every `trace_*` free function.
    #[derive(Clone, Debug, Default)]
    struct Traced {
        code: String,
        temp_variables: usize,
        outputs: usize,
    }

    /// A `subtemplate name -> template file path` pair, matching one entry of
    /// `cricket::GenOptions::subtemplates`.
    #[derive(Clone, Debug, Default)]
    struct Subtemplate {
        name: String,
        path: String,
    }

    /// Mirrors `cricket::GenResult`, with `data` serialized to JSON (`data_json`).
    #[derive(Clone, Debug, Default)]
    struct GenResult {
        source: String,
        data_json: String,
        robot_name: String,
        dimension: usize,
        n_spheres: usize,
    }

    unsafe extern "C++" {
        include!("shim.h");

        /// Opaque handle to a `cricket::RobotInfo` (bound directly to the real upstream type,
        /// hence the `namespace` override -- not a wrapper). Every accessor below takes this by
        /// shared reference: `RobotInfo::json`/`dof_to_joint_names` are declared non-`const`
        /// upstream but don't mutate any observable state (verified by reading
        /// `robot_info.cc`), so the shim `const_cast`s internally rather than forcing every read
        /// through `Pin<&mut _>`. `guess_self_collisions` is a real mutator and is the one
        /// function that needs it.
        #[namespace = "cricket"]
        type RobotInfo;

        /// Parses a URDF (+ optional SRDF) into a `RobotInfo`, matching the
        /// `cricket::RobotInfo` constructor. Empty strings select the constructor's `nullopt`
        /// defaults: no SRDF means self-collisions are guessed rather than read from file, and
        /// no end effector means the URDF's most distal link is used.
        fn robot_info_new(
            urdf: &str,
            srdf: &str,
            end_effector: &str,
        ) -> Result<UniquePtr<RobotInfo>>;

        fn dimension(info: &RobotInfo) -> usize;
        fn n_spheres(info: &RobotInfo) -> usize;
        fn min_radius(info: &RobotInfo) -> f32;
        fn max_radius(info: &RobotInfo) -> f32;
        fn min_radius_mobile(info: &RobotInfo) -> f32;
        fn max_radius_mobile(info: &RobotInfo) -> f32;
        fn min_bounding_radius_mobile(info: &RobotInfo) -> f32;
        fn max_bounding_radius_mobile(info: &RobotInfo) -> f32;
        fn base_position(info: &RobotInfo) -> [f32; 3];
        fn end_effector_name(info: &RobotInfo) -> String;
        fn end_effector_index(info: &RobotInfo) -> usize;

        /// Mirrors `RobotInfo::json`, dumped to a JSON string.
        fn robot_info_json(info: &RobotInfo, has_bounds: bool, bounds: Bounds) -> Result<String>;
        fn dof_to_joint_names(info: &RobotInfo) -> Vec<String>;
        fn spheres(info: &RobotInfo) -> Vec<SphereInfo>;
        fn bounding_spheres(info: &RobotInfo) -> Vec<BoundingSphere>;
        fn allowed_link_pairs(info: &RobotInfo) -> Vec<LinkPair>;
        fn per_link_spheres(info: &RobotInfo) -> Vec<LinkSpheres>;
        fn links_with_geometry(info: &RobotInfo) -> Vec<usize>;
        fn bounding_sphere_index(info: &RobotInfo) -> Vec<usize>;

        /// Re-runs `RobotInfo::guess_self_collisions`, discarding whatever `allowed_link_pairs`
        /// the constructor (or a previous call) computed.
        fn guess_self_collisions(info: Pin<&mut RobotInfo>, n: usize) -> Result<()>;

        fn trace_sphere_cc_fk(
            info: &RobotInfo,
            language: &str,
            spheres: bool,
            bounding_spheres: bool,
            fk: bool,
        ) -> Result<Traced>;
        fn trace_map_to_configuration(
            info: &RobotInfo,
            language: &str,
            has_bounds: bool,
            bounds: Bounds,
        ) -> Result<Traced>;
        fn trace_interpolate(info: &RobotInfo, language: &str) -> Result<Traced>;
        fn trace_interpolate_block(info: &RobotInfo, language: &str) -> Result<Traced>;
        fn trace_distance(info: &RobotInfo, language: &str) -> Result<Traced>;

        /// The same URDF -> traced FK/CC code -> inja template render pipeline as
        /// `cricket::generate_robot_source`, kept as a one-shot convenience entry point rather
        /// than making every caller re-derive it from the pieces above.
        #[allow(clippy::too_many_arguments)]
        fn generate_robot_source(
            urdf: &str,
            srdf: &str,
            end_effector: &str,
            template_path: &str,
            subtemplates: Vec<Subtemplate>,
            language: &str,
            has_bounds: bool,
            bounds: Bounds,
            extra_data_json: &str,
        ) -> Result<GenResult>;
    }
}

pub use bridge::*;
