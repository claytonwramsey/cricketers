use std::path::Path;

use crate::{
    error::{Result, path_to_str},
    robot_info::{Bounds, Language},
};

/// Options for [`generate_robot_source`], mirroring `cricket::GenOptions`.
#[derive(Default)]
pub struct GenOptions<'a> {
    /// The path to the robot's semantic description file, providing skipped collisions. If
    /// `None`, cricket estimates the non-colliding joints automatically.
    pub srdf: Option<&'a Path>,
    /// The end effector link's name. If `None`, the URDF's most distal link is used.
    pub end_effector: Option<&'a str>,
    /// The inja template to render. If `None`, cricket's embedded default C++ template is used.
    pub template_path: Option<&'a Path>,
    /// Named subtemplates the main template can `{% include %}`.
    pub subtemplates: &'a [(&'a str, &'a Path)],
    pub language: Language,
    pub bounds: Option<Bounds>,
    /// Extra data merged into the template's data context before rendering (e.g. pass
    /// `{"name": "Panda"}` to populate [`GenResult::robot_name`]).
    pub extra_data: Option<serde_json::Value>,
}

/// The rendered source, plus the metadata cricket derived about the robot along the way.
#[derive(Debug)]
pub struct GenResult {
    pub source: String,
    /// The full merged template data context (bounds, joint topology, every traced code
    /// fragment, ...), as parsed JSON.
    pub data: serde_json::Value,
    pub robot_name: String,
    pub dimension: usize,
    pub n_spheres: usize,
}

/// Runs the URDF -> traced FK/CC code -> inja template render pipeline in one shot, matching
/// `cricket::generate_robot_source`. This is the same pipeline [`RobotInfo`](crate::RobotInfo)'s
/// `trace_*` methods let you assemble by hand; use this when you just want the end result.
pub fn generate_robot_source(urdf: &Path, opts: &GenOptions) -> Result<GenResult> {
    let urdf = path_to_str(urdf)?;
    let srdf = opts.srdf.map(path_to_str).transpose()?.unwrap_or("");
    let template_path = opts
        .template_path
        .map(path_to_str)
        .transpose()?
        .unwrap_or("");

    let subtemplates = opts
        .subtemplates
        .iter()
        .map(|(name, path)| {
            Ok(cricket_sys::Subtemplate {
                name: (*name).to_owned(),
                path: path_to_str(path)?.to_owned(),
            })
        })
        .collect::<Result<Vec<_>>>()?;

    let extra_data_json = opts
        .extra_data
        .as_ref()
        .map(serde_json::Value::to_string)
        .unwrap_or_default();

    let raw = cricket_sys::generate_robot_source(
        urdf,
        srdf,
        opts.end_effector.unwrap_or(""),
        template_path,
        subtemplates,
        opts.language.as_str(),
        opts.bounds.is_some(),
        opts.bounds.unwrap_or_default().into(),
        &extra_data_json,
    )?;

    Ok(GenResult {
        source: raw.source,
        data: serde_json::from_str(&raw.data_json)
            .expect("cricket::generate_robot_source always returns an object"),
        robot_name: raw.robot_name,
        dimension: raw.dimension,
        n_spheres: raw.n_spheres,
    })
}
