//! Idiomatic Rust bindings for [cricket](https://github.com/CoMMALab/cricket), built on top of
//! the raw `cricket-sys` bridge. Where `cricket-sys` mirrors the C++ API almost literally (flat
//! FFI-safe structs, `&str` in place of `Option<&str>`, `cxx::Exception`), this crate translates
//! that surface into native Rust: `Option`/`Result`, owned `HashMap`/`BTreeSet`/`Vec<Vec<_>>`
//! containers, a `Language` enum in place of stringly-typed `"c++"`/`"rust"`, and parsed
//! `serde_json::Value` in place of raw JSON strings.

mod codegen;
mod error;
mod robot_info;

pub use codegen::{GenOptions, GenResult, generate_robot_source};
pub use error::{Error, Result};
pub use robot_info::{Bounds, Isometry3, Language, RobotInfo, Sphere, Traced};

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn resources_dir() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../cricket-sys/vendor/cricket/resources")
    }

    fn panda_robot_info() -> RobotInfo {
        let resources = resources_dir();
        RobotInfo::new(
            &resources.join("panda/panda_spherized.urdf"),
            Some(&resources.join("panda/panda.srdf")),
            Some("panda_grasptarget"),
        )
        .expect("RobotInfo::new failed")
    }

    #[test]
    fn robot_info_exposes_native_containers() {
        let info = panda_robot_info();

        assert!(info.dimension() > 0);
        assert_eq!(info.end_effector_name(), "panda_grasptarget");

        let spheres = info.spheres();
        assert_eq!(spheres.len(), info.n_spheres());
        for sphere in &spheres {
            assert!(sphere.pose.translation.iter().all(|x| x.is_finite()));
        }

        let bounding = info.bounding_spheres();
        assert!(!bounding.is_empty());
        // HashMap keyed by frame index -- every key should be a real frame with geometry.
        let links_with_geometry: std::collections::HashSet<_> =
            info.links_with_geometry().into_iter().collect();
        for frame_index in bounding.keys() {
            assert!(links_with_geometry.contains(frame_index));
        }

        let per_link = info.per_link_spheres();
        assert_eq!(
            per_link.len(),
            info.links_with_geometry().len().max(per_link.len())
        );
        assert!(!per_link.is_empty());

        let pairs = info.allowed_link_pairs();
        for (a, b) in &pairs {
            assert!(a <= b, "allowed_link_pairs should stay normalized (a <= b)");
        }
    }

    #[test]
    fn robot_info_metadata_is_parsed_json() {
        let info = panda_robot_info();
        let metadata = info.metadata(None).expect("metadata failed");
        assert_eq!(metadata["n_q"], info.dimension());
        assert_eq!(metadata["end_effector"], "panda_grasptarget");
    }

    #[test]
    fn trace_functions_use_the_language_enum() {
        let info = panda_robot_info();

        let cpp = info
            .trace_sphere_cc_fk(Language::Cpp, false, false, true)
            .expect("c++ trace failed");
        assert!(!cpp.code.is_empty());

        let rust = info
            .trace_sphere_cc_fk(Language::Rust, false, false, true)
            .expect("rust trace failed");
        assert!(!rust.code.is_empty());
        assert_ne!(cpp.code, rust.code);
    }

    #[test]
    fn guess_self_collisions_can_be_rerun() {
        let mut info = panda_robot_info();
        let before = info.allowed_link_pairs().len();
        info.guess_self_collisions(100)
            .expect("guess_self_collisions failed");
        assert!(info.allowed_link_pairs().len() <= before);
    }

    #[test]
    fn generate_robot_source_end_to_end() {
        let resources = resources_dir();
        let result = generate_robot_source(
            &resources.join("panda/panda_spherized.urdf"),
            &GenOptions {
                srdf: Some(&resources.join("panda/panda.srdf")),
                end_effector: Some("panda_grasptarget"),
                extra_data: Some(serde_json::json!({"name": "Panda", "resolution": 32})),
                ..Default::default()
            },
        )
        .expect("generate_robot_source failed");

        assert_eq!(result.robot_name, "Panda");
        assert!(result.n_spheres > 0);
        assert!(result.source.contains("namespace"));
        assert_eq!(result.data["n_q"], result.dimension);
    }

    #[test]
    fn reports_missing_urdf_as_a_typed_error() {
        let err =
            generate_robot_source(Path::new("/nonexistent/robot.urdf"), &GenOptions::default())
                .expect_err("expected an error for a missing URDF");
        assert!(matches!(err, Error::Cricket(_)));
        assert!(!err.to_string().is_empty());
    }
}
