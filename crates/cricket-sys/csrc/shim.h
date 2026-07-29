#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /// A pair of nul-terminated `subtemplate name -> template file path` strings, matching
    /// cricket::GenOptions::subtemplates.
    typedef struct CricketSubtemplate
    {
        const char *name;
        const char *path;
    } CricketSubtemplate;

    /// Optional Cartesian bounds for FreeFlyer / Planar joints, matching cricket::Bounds.
    typedef struct CricketBounds
    {
        double lower[3];
        double upper[3];
    } CricketBounds;

    typedef struct CricketGenSuccess
    {
        char *source;
        /// The full merged data object (including all traced code fragments), serialized as JSON.
        char *data_json;
        char *robot_name;
        size_t dimension;
        size_t n_spheres;
    } CricketGenSuccess;

    typedef union CricketGenResultData
    {
        /// Valid iff CricketGenResult::success == 0.
        char *error_message;
        /// Valid iff CricketGenResult::success == 1.
        CricketGenSuccess success;
    } CricketGenResultData;

    typedef struct CricketGenResult
    {
        /// 0 on failure, 1 on success -- selects which member of `data` is valid.
        int success;
        CricketGenResultData data;
    } CricketGenResult;

    /// Traces a robot's forward kinematics/collision code from a URDF (+ optional SRDF) and
    /// renders it through an inja template, mirroring cricket::generate_robot_source.
    ///
    /// `urdf_path` is required; all other pointer arguments may be NULL for their documented
    /// default. `template_path` NULL or "" selects cricket's embedded default C++ template.
    /// `extra_data_json` NULL or "" is treated as "{}"; its contents are merged into the
    /// template's data context before rendering (e.g. pass {"name": "Panda"} to populate
    /// CricketGenResult::data.success.robot_name).
    ///
    /// The returned CricketGenResult must be released with cricket_free_gen_result.
    CricketGenResult cricket_generate_robot_source(
        const char *urdf_path,
        const char *srdf_path,
        const char *end_effector,
        const char *template_path,
        const CricketSubtemplate *subtemplates,
        size_t subtemplate_count,
        const char *language,
        const CricketBounds *bounds,
        const char *extra_data_json);

    void cricket_free_gen_result(CricketGenResult *result);

#ifdef __cplusplus
}
#endif
