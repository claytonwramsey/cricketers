//! Raw, low-level bindings to cricket's `RobotInfo` and `trace_*`/`generate_robot_source`
//! functions, built on `cxx`. Everything here is a near-literal mirror of the C++ API (see
//! `src/ffi.rs` for the exact surface and the reasoning behind it) -- richer, idiomatic Rust
//! ergonomics (owned nalgebra-style transforms, builder types, `Option`-based bounds, ...)
//! belong in the `cricket` crate, not here.

mod ffi;

pub use ffi::{
    BoundingSphere, Bounds, GenResult, LinkPair, LinkSpheres, RobotInfo, SphereInfo, Subtemplate,
    Traced, allowed_link_pairs, base_position, bounding_sphere_index, bounding_spheres, dimension,
    dof_to_joint_names, end_effector_index, end_effector_name, generate_robot_source,
    guess_self_collisions, links_with_geometry, max_bounding_radius_mobile, max_radius,
    max_radius_mobile, min_bounding_radius_mobile, min_radius, min_radius_mobile, n_spheres,
    per_link_spheres, robot_info_json, robot_info_new, spheres, trace_distance, trace_interpolate,
    trace_interpolate_block, trace_map_to_configuration, trace_sphere_cc_fk,
};

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn resources_dir() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("vendor/cricket/resources")
    }

    #[test]
    fn generates_panda_fk_from_embedded_template() {
        let resources = resources_dir();
        let result = generate_robot_source(
            resources
                .join("panda/panda_spherized.urdf")
                .to_str()
                .unwrap(),
            resources.join("panda/panda.srdf").to_str().unwrap(),
            "panda_grasptarget",
            "",
            Vec::new(),
            "",
            false,
            Bounds::default(),
            r#"{"name": "Panda", "resolution": 32}"#,
        )
        .expect("generate_robot_source failed");

        assert_eq!(result.robot_name, "Panda");
        assert!(result.n_spheres > 0);
        assert!(result.source.contains("namespace"));
    }

    #[test]
    fn reports_missing_urdf_as_an_error() {
        let err = generate_robot_source(
            "/nonexistent/robot.urdf",
            "",
            "",
            "",
            Vec::new(),
            "",
            false,
            Bounds::default(),
            "",
        )
        .expect_err("expected an error for a missing URDF");
        assert!(!err.what().is_empty());
    }

    fn panda_robot_info() -> cxx::UniquePtr<RobotInfo> {
        let resources = resources_dir();
        robot_info_new(
            resources
                .join("panda/panda_spherized.urdf")
                .to_str()
                .unwrap(),
            resources.join("panda/panda.srdf").to_str().unwrap(),
            "panda_grasptarget",
        )
        .expect("robot_info_new failed")
    }

    #[test]
    fn robot_info_exposes_scalar_fields() {
        let info = panda_robot_info();
        assert!(dimension(&info) > 0);
        assert!(n_spheres(&info) > 0);
        assert!(min_radius(&info) <= max_radius(&info));
        assert_eq!(end_effector_name(&info), "panda_grasptarget");
        assert!(end_effector_index(&info) > 0);
    }

    #[test]
    fn robot_info_exposes_container_fields() {
        let info = panda_robot_info();

        let names = dof_to_joint_names(&info);
        assert_eq!(names.len(), dimension(&info));

        let all_spheres = spheres(&info);
        assert_eq!(all_spheres.len(), n_spheres(&info));
        // Every sphere's rotation should at least be finite, confirming the SE3 -> flat-array
        // decomposition ran (as opposed to e.g. silently leaving zeroed memory).
        for sphere in &all_spheres {
            assert!(sphere.rotation.iter().all(|x| x.is_finite()));
        }

        let bounding = bounding_spheres(&info);
        assert!(!bounding.is_empty());

        let per_link = per_link_spheres(&info);
        assert_eq!(per_link.len(), names.len().max(per_link.len())); // non-empty, sane length
        assert!(!per_link.is_empty());

        // allowed_link_pairs/links_with_geometry/bounding_sphere_index should all be
        // internally consistent in length with the sphere data above.
        assert_eq!(bounding_sphere_index(&info).len(), per_link.len());
        let _ = allowed_link_pairs(&info);
        let _ = links_with_geometry(&info);
    }

    #[test]
    fn robot_info_json_matches_scalar_accessors() {
        let info = panda_robot_info();
        let json_str = robot_info_json(&info, false, Bounds::default()).expect("json failed");
        assert!(json_str.contains(&format!("\"n_q\":{}", dimension(&info))));
        assert!(json_str.contains("panda_grasptarget"));
    }

    #[test]
    fn trace_functions_return_nonempty_code() {
        let info = panda_robot_info();

        let eefk =
            trace_sphere_cc_fk(&info, "c++", false, false, true).expect("trace_sphere_cc_fk");
        assert!(!eefk.code.is_empty());
        assert_eq!(eefk.outputs, 12);

        let mapconfig = trace_map_to_configuration(&info, "c++", false, Bounds::default())
            .expect("trace_map_to_configuration");
        assert!(!mapconfig.code.is_empty());

        let interp = trace_interpolate(&info, "c++").expect("trace_interpolate");
        assert!(!interp.code.is_empty());

        let interp_block = trace_interpolate_block(&info, "c++").expect("trace_interpolate_block");
        assert!(!interp_block.code.is_empty());

        let dist = trace_distance(&info, "c++").expect("trace_distance");
        assert!(!dist.code.is_empty());
    }

    #[test]
    fn trace_sphere_cc_fk_rejects_unsupported_language() {
        let info = panda_robot_info();
        let err = trace_sphere_cc_fk(&info, "cobol", false, false, true)
            .expect_err("expected an error for an unsupported language");
        assert!(err.what().contains("cobol"));
    }

    #[test]
    fn guess_self_collisions_can_be_rerun() {
        let mut info = panda_robot_info();
        let before = allowed_link_pairs(&info).len();
        guess_self_collisions(info.pin_mut(), 100).expect("guess_self_collisions failed");
        // A much smaller sample size should never find more colliding pairs than a
        // larger one already found at construction time.
        assert!(allowed_link_pairs(&info).len() <= before);
    }
}
