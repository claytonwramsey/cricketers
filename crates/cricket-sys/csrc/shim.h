#pragma once

// `cricket/robot_info.hh` (bringing in `cricket::RobotInfo`, the bridge's `#[namespace =
// "cricket"] type RobotInfo` target) must be included before the generated
// `cricket-sys/src/ffi.rs.h` below: that header is also `include!`-d directly *into*
// `ffi.rs.h`/`ffi.rs.cc` (this is that same file, doing double duty as both the C++
// implementation of the bridge and a normal header), which re-enters this file recursively:
// the `#pragma once` guard here breaks the cycle, but only by skipping the *second* attempt to
// include this header -- cxx's own generated forward declaration for RobotInfo
// (`namespace cricket { using RobotInfo = ::cricket::RobotInfo; }`) still runs during that
// first, nested pass, so `cricket::RobotInfo` needs to already be a complete type by then.
#include <cricket/codegen.hh>
#include <cricket/robot_info.hh>

// Declares `cricket_ffi::{Bounds, SphereInfo, BoundingSphere, LinkPair, LinkSpheres, Traced,
// Subtemplate, GenResult}` and pulls in `rust::{String, Str, Vec}` via `rust/cxx.h`.
#include "cricket-sys/src/ffi.rs.h"

#include <array>
#include <cstddef>
#include <memory>

namespace cricket_ffi
{
    /// Local convenience alias only -- the bridge binds directly to `cricket::RobotInfo` (see
    /// the `#[namespace = "cricket"]` override in `src/ffi.rs`), so unlike every other type in
    /// this header, this name never appears in cxx's own generated code.
    using RobotInfo = cricket::RobotInfo;

    std::unique_ptr<RobotInfo> robot_info_new(rust::Str urdf, rust::Str srdf, rust::Str end_effector);

    std::size_t dimension(const RobotInfo &info);
    std::size_t n_spheres(const RobotInfo &info);
    float min_radius(const RobotInfo &info);
    float max_radius(const RobotInfo &info);
    float min_radius_mobile(const RobotInfo &info);
    float max_radius_mobile(const RobotInfo &info);
    float min_bounding_radius_mobile(const RobotInfo &info);
    float max_bounding_radius_mobile(const RobotInfo &info);
    std::array<float, 3> base_position(const RobotInfo &info);
    rust::String end_effector_name(const RobotInfo &info);
    std::size_t end_effector_index(const RobotInfo &info);

    rust::String robot_info_json(const RobotInfo &info, bool has_bounds, Bounds bounds);
    rust::Vec<rust::String> dof_to_joint_names(const RobotInfo &info);
    rust::Vec<SphereInfo> spheres(const RobotInfo &info);
    rust::Vec<BoundingSphere> bounding_spheres(const RobotInfo &info);
    rust::Vec<LinkPair> allowed_link_pairs(const RobotInfo &info);
    rust::Vec<LinkSpheres> per_link_spheres(const RobotInfo &info);
    rust::Vec<std::size_t> links_with_geometry(const RobotInfo &info);
    rust::Vec<std::size_t> bounding_sphere_index(const RobotInfo &info);

    void guess_self_collisions(RobotInfo &info, std::size_t n);

    Traced trace_sphere_cc_fk(
        const RobotInfo &info,
        rust::Str language,
        bool spheres,
        bool bounding_spheres,
        bool fk);
    Traced trace_map_to_configuration(
        const RobotInfo &info,
        rust::Str language,
        bool has_bounds,
        Bounds bounds);
    Traced trace_interpolate(const RobotInfo &info, rust::Str language);
    Traced trace_interpolate_block(const RobotInfo &info, rust::Str language);
    Traced trace_distance(const RobotInfo &info, rust::Str language);

    GenResult generate_robot_source(
        rust::Str urdf,
        rust::Str srdf,
        rust::Str end_effector,
        rust::Str template_path,
        rust::Vec<Subtemplate> subtemplates,
        rust::Str language,
        bool has_bounds,
        Bounds bounds,
        rust::Str extra_data_json);
}  // namespace cricket_ffi
