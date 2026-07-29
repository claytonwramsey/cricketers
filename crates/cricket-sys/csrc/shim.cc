#include "shim.h"

#include <cricket/codegen.hh>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>

namespace
{
    char *dup_string(const std::string &s)
    {
        char *out = static_cast<char *>(std::malloc(s.size() + 1));
        std::memcpy(out, s.data(), s.size());
        out[s.size()] = '\0';
        return out;
    }

    CricketGenResult make_error(const std::string &message)
    {
        CricketGenResult result{};
        result.success = 0;
        result.data.error_message = dup_string(message);
        return result;
    }
}  // namespace

extern "C" CricketGenResult cricket_generate_robot_source(
    const char *urdf_path,
    const char *srdf_path,
    const char *end_effector,
    const char *template_path,
    const CricketSubtemplate *subtemplates,
    size_t subtemplate_count,
    const char *language,
    const CricketBounds *bounds,
    const char *extra_data_json)
{
    try
    {
        cricket::GenOptions opts;
        opts.urdf = std::filesystem::path(urdf_path);

        if (srdf_path != nullptr)
        {
            opts.srdf = std::filesystem::path(srdf_path);
        }
        if (end_effector != nullptr)
        {
            opts.end_effector = std::string(end_effector);
        }
        if (template_path != nullptr)
        {
            opts.template_path = std::filesystem::path(template_path);
        }
        for (size_t i = 0; i < subtemplate_count; ++i)
        {
            opts.subtemplates[std::string(subtemplates[i].name)] =
                std::filesystem::path(subtemplates[i].path);
        }
        if (language != nullptr)
        {
            opts.language = std::string(language);
        }
        if (bounds != nullptr)
        {
            cricket::Bounds b;
            b.lower = Eigen::Vector3d(bounds->lower[0], bounds->lower[1], bounds->lower[2]);
            b.upper = Eigen::Vector3d(bounds->upper[0], bounds->upper[1], bounds->upper[2]);
            opts.bounds = b;
        }
        opts.data = (extra_data_json != nullptr && extra_data_json[0] != '\0')
            ? nlohmann::json::parse(extra_data_json)
            : nlohmann::json::object();

        cricket::GenResult gen = cricket::generate_robot_source(opts);

        CricketGenResult result{};
        result.success = 1;
        result.data.success.source = dup_string(gen.source);
        result.data.success.data_json = dup_string(gen.data.dump());
        result.data.success.robot_name = dup_string(gen.robot_name);
        result.data.success.dimension = gen.dimension;
        result.data.success.n_spheres = gen.n_spheres;
        return result;
    }
    catch (const std::exception &e)
    {
        return make_error(e.what());
    }
    catch (...)
    {
        return make_error("unknown C++ exception in cricket_generate_robot_source");
    }
}

extern "C" void cricket_free_gen_result(CricketGenResult *result)
{
    if (result == nullptr)
    {
        return;
    }
    if (result->success)
    {
        std::free(result->data.success.source);
        std::free(result->data.success.data_json);
        std::free(result->data.success.robot_name);
    }
    else
    {
        std::free(result->data.error_message);
    }
    *result = CricketGenResult{};
}
