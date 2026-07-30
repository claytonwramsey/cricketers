#include "shim.h"

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <utility>

// None of the functions below catch C++ exceptions: every one of them is declared `Result<_>`
// on the Rust side of `src/ffi.rs`, so `cxx` already wraps this translation unit's generated
// call sites in a `try`/`catch` that turns a thrown `std::exception` into `Err(cxx::Exception)`.

namespace cricket_ffi
{
    namespace
    {
        std::string to_std_string(rust::Str s) { return std::string(s.data(), s.size()); }

        std::optional<std::string> to_opt_string(rust::Str s)
        {
            if (s.empty())
            {
                return std::nullopt;
            }
            return to_std_string(s);
        }

        std::optional<std::filesystem::path> to_opt_path(rust::Str s)
        {
            if (s.empty())
            {
                return std::nullopt;
            }
            return std::filesystem::path(to_std_string(s));
        }

        cricket::Bounds to_cricket_bounds(const Bounds &b)
        {
            cricket::Bounds out;
            out.lower = Eigen::Vector3d(b.lower[0], b.lower[1], b.lower[2]);
            out.upper = Eigen::Vector3d(b.upper[0], b.upper[1], b.upper[2]);
            return out;
        }

        std::optional<cricket::Bounds> to_opt_bounds(bool has_bounds, const Bounds &b)
        {
            if (!has_bounds)
            {
                return std::nullopt;
            }
            return to_cricket_bounds(b);
        }

        SphereInfo to_ffi_sphere(const cricket::SphereInfo &s)
        {
            const auto &t = s.relative.translation();
            const auto &r = s.relative.rotation();

            SphereInfo out{};
            out.geom_index = s.geom_index;
            out.radius = s.radius;
            out.parent_joint = s.parent_joint;
            out.parent_frame = s.parent_frame;
            out.translation = {t[0], t[1], t[2]};
            // Row-major, matching the field doc on the Rust side.
            out.rotation = {
                r(0, 0), r(0, 1), r(0, 2),
                r(1, 0), r(1, 1), r(1, 2),
                r(2, 0), r(2, 1), r(2, 2),
            };
            return out;
        }

        Traced to_ffi_traced(const cricket::Traced &t)
        {
            return Traced{rust::String(t.code), t.temp_variables, t.outputs};
        }
    }  // namespace

    std::unique_ptr<RobotInfo> robot_info_new(rust::Str urdf, rust::Str srdf, rust::Str end_effector)
    {
        std::filesystem::path urdf_path(to_std_string(urdf));
        return std::make_unique<RobotInfo>(urdf_path, to_opt_path(srdf), to_opt_string(end_effector));
    }

    std::size_t dimension(const RobotInfo &info) { return info.model.nq; }
    std::size_t n_spheres(const RobotInfo &info) { return info.spheres.size(); }
    float min_radius(const RobotInfo &info) { return info.min_radius; }
    float max_radius(const RobotInfo &info) { return info.max_radius; }
    float min_radius_mobile(const RobotInfo &info) { return info.min_radius_mobile; }
    float max_radius_mobile(const RobotInfo &info) { return info.max_radius_mobile; }
    float min_bounding_radius_mobile(const RobotInfo &info) { return info.min_bounding_radius_mobile; }
    float max_bounding_radius_mobile(const RobotInfo &info) { return info.max_bounding_radius_mobile; }

    std::array<float, 3> base_position(const RobotInfo &info)
    {
        return {info.base_position[0], info.base_position[1], info.base_position[2]};
    }

    rust::String end_effector_name(const RobotInfo &info) { return rust::String(info.end_effector_name); }
    std::size_t end_effector_index(const RobotInfo &info) { return info.end_effector_index; }

    rust::String robot_info_json(const RobotInfo &info, bool has_bounds, Bounds bounds)
    {
        // `RobotInfo::json` is declared non-const upstream but only reads fields populated by
        // the constructor -- see the doc comment on `type RobotInfo` in `src/ffi.rs`.
        auto &mut_info = const_cast<RobotInfo &>(info);
        return rust::String(mut_info.json(to_opt_bounds(has_bounds, bounds)).dump());
    }

    rust::Vec<rust::String> dof_to_joint_names(const RobotInfo &info)
    {
        auto &mut_info = const_cast<RobotInfo &>(info);
        rust::Vec<rust::String> out;
        for (const auto &name : mut_info.dof_to_joint_names())
        {
            out.push_back(rust::String(name));
        }
        return out;
    }

    rust::Vec<SphereInfo> spheres(const RobotInfo &info)
    {
        rust::Vec<SphereInfo> out;
        for (const auto &s : info.spheres)
        {
            out.push_back(to_ffi_sphere(s));
        }
        return out;
    }

    rust::Vec<BoundingSphere> bounding_spheres(const RobotInfo &info)
    {
        rust::Vec<BoundingSphere> out;
        for (const auto &[frame, sphere] : info.bounding_spheres)
        {
            out.push_back(BoundingSphere{frame, to_ffi_sphere(sphere)});
        }
        return out;
    }

    rust::Vec<LinkPair> allowed_link_pairs(const RobotInfo &info)
    {
        rust::Vec<LinkPair> out;
        for (const auto &[first, second] : info.allowed_link_pairs)
        {
            out.push_back(LinkPair{first, second});
        }
        return out;
    }

    rust::Vec<LinkSpheres> per_link_spheres(const RobotInfo &info)
    {
        rust::Vec<LinkSpheres> out;
        for (std::size_t i = 0; i < info.per_link_spheres.size(); ++i)
        {
            LinkSpheres entry;
            entry.link_index = i;
            for (auto idx : info.per_link_spheres[i])
            {
                entry.sphere_indices.push_back(idx);
            }
            out.push_back(std::move(entry));
        }
        return out;
    }

    rust::Vec<std::size_t> links_with_geometry(const RobotInfo &info)
    {
        rust::Vec<std::size_t> out;
        for (auto v : info.links_with_geometry)
        {
            out.push_back(v);
        }
        return out;
    }

    rust::Vec<std::size_t> bounding_sphere_index(const RobotInfo &info)
    {
        rust::Vec<std::size_t> out;
        for (auto v : info.bounding_sphere_index)
        {
            out.push_back(v);
        }
        return out;
    }

    void guess_self_collisions(RobotInfo &info, std::size_t n) { info.guess_self_collisions(n); }

    Traced trace_sphere_cc_fk(
        const RobotInfo &info,
        rust::Str language,
        bool spheres,
        bool bounding_spheres,
        bool fk)
    {
        return to_ffi_traced(
            cricket::trace_sphere_cc_fk(info, to_std_string(language), spheres, bounding_spheres, fk));
    }

    Traced trace_map_to_configuration(
        const RobotInfo &info,
        rust::Str language,
        bool has_bounds,
        Bounds bounds)
    {
        return to_ffi_traced(
            cricket::trace_map_to_configuration(
                info.model, to_std_string(language), to_opt_bounds(has_bounds, bounds)));
    }

    Traced trace_interpolate(const RobotInfo &info, rust::Str language)
    {
        return to_ffi_traced(cricket::trace_interpolate(info.model, to_std_string(language)));
    }

    Traced trace_interpolate_block(const RobotInfo &info, rust::Str language)
    {
        return to_ffi_traced(cricket::trace_interpolate_block(info.model, to_std_string(language)));
    }

    Traced trace_distance(const RobotInfo &info, rust::Str language)
    {
        return to_ffi_traced(cricket::trace_distance(info.model, to_std_string(language)));
    }

    GenResult generate_robot_source(
        rust::Str urdf,
        rust::Str srdf,
        rust::Str end_effector,
        rust::Str template_path,
        rust::Vec<Subtemplate> subtemplates,
        rust::Str language,
        bool has_bounds,
        Bounds bounds,
        rust::Str extra_data_json)
    {
        cricket::GenOptions opts;
        opts.urdf = std::filesystem::path(to_std_string(urdf));
        opts.srdf = to_opt_path(srdf);
        opts.end_effector = to_opt_string(end_effector);
        if (!template_path.empty())
        {
            opts.template_path = std::filesystem::path(to_std_string(template_path));
        }
        for (const auto &st : subtemplates)
        {
            opts.subtemplates[to_std_string(st.name)] = std::filesystem::path(to_std_string(st.path));
        }
        if (!language.empty())
        {
            opts.language = to_std_string(language);
        }
        opts.bounds = to_opt_bounds(has_bounds, bounds);
        opts.data = extra_data_json.empty() ? nlohmann::json::object()
                                             : nlohmann::json::parse(to_std_string(extra_data_json));

        cricket::GenResult gen = cricket::generate_robot_source(opts);

        GenResult out;
        out.source = rust::String(gen.source);
        out.data_json = rust::String(gen.data.dump());
        out.robot_name = rust::String(gen.robot_name);
        out.dimension = gen.dimension;
        out.n_spheres = gen.n_spheres;
        return out;
    }
}  // namespace cricket_ffi
