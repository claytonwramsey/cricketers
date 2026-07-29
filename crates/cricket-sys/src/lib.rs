use std::{
    ffi::{CStr, CString},
    os::raw::{c_char, c_int},
    path::Path,
};

#[repr(C)]
struct RawSubtemplate {
    name: *const c_char,
    path: *const c_char,
}

/// Optional Cartesian bounds for FreeFlyer / Planar joints.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    pub lower: [f64; 3],
    pub upper: [f64; 3],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RawGenSuccess {
    source: *mut c_char,
    data_json: *mut c_char,
    robot_name: *mut c_char,
    dimension: usize,
    n_spheres: usize,
}

#[repr(C)]
union RawGenResultData {
    /// Valid iff `RawGenResult::success == 0`.
    error_message: *mut c_char,
    /// Valid iff `RawGenResult::success != 0`.
    success: RawGenSuccess,
}

#[repr(C)]
struct RawGenResult {
    success: c_int,
    data: RawGenResultData,
}

unsafe extern "C" {
    fn cricket_generate_robot_source(
        urdf_path: *const c_char,
        srdf_path: *const c_char,
        end_effector: *const c_char,
        template_path: *const c_char,
        subtemplates: *const RawSubtemplate,
        subtemplate_count: usize,
        language: *const c_char,
        bounds: *const Bounds,
        extra_data_json: *const c_char,
    ) -> RawGenResult;

    fn cricket_free_gen_result(result: *mut RawGenResult);
}

/// Traces a robot's forward kinematics/collision code from a URDF (+ optional SRDF) and renders
/// it through an inja template, mirroring cricket's `generate_robot_source`. `template_path`
/// `None` selects cricket's embedded default C++ template.
#[derive(Default)]
pub struct GenOptions<'a> {
    /// The path to the robot's semantic description file, providing skipped collisions.
    /// If `None`, `cricket` will automatically estimate the non-colliding joints.
    pub srdf: Option<&'a Path>,
    /// The end effector link's name.
    pub end_effector: Option<&'a str>,
    /// The template.
    pub template_path: Option<&'a Path>,
    /// Paths to subtemplates.
    pub subtemplates: &'a [(&'a str, &'a Path)],
    /// The output language.
    pub language: Option<&'a str>,
    /// The bounds on the free-flying joints.
    pub bounds: Option<Bounds>,
    /// Extra data merged into the template's data context before rendering (e.g. pass
    /// `{"name": "Panda"}` to populate [`GenResult::robot_name`]).
    pub extra_data_json: Option<&'a str>,
}

#[derive(Debug)]
pub struct GenResult {
    pub source: String,
    /// The full merged data object (including all traced code fragments), as JSON.
    pub data_json: String,
    pub robot_name: String,
    pub dimension: usize,
    pub n_spheres: usize,
}

fn path_to_cstring(path: &Path) -> Result<CString, String> {
    let s = path
        .to_str()
        .ok_or_else(|| format!("path {} is not valid UTF-8", path.display()))?;
    CString::new(s).map_err(|e| e.to_string())
}

fn opt_cstring(s: Option<&str>) -> Result<Option<CString>, String> {
    s.map(|s| CString::new(s).map_err(|e| e.to_string()))
        .transpose()
}

/// # Safety
/// This function is safe: it owns every pointer it hands to the FFI call for the call's
/// duration, and frees the result before returning.
pub fn generate_robot_source(urdf: &Path, opts: &GenOptions) -> Result<GenResult, String> {
    let urdf = path_to_cstring(urdf)?;
    let srdf = opts.srdf.map(path_to_cstring).transpose()?;
    let end_effector = opt_cstring(opts.end_effector)?;
    let template_path = opts.template_path.map(path_to_cstring).transpose()?;
    let language = opt_cstring(opts.language)?;
    let extra_data_json = opt_cstring(opts.extra_data_json)?;

    let subtemplate_cstrings = opts
        .subtemplates
        .iter()
        .map(|(name, path)| {
            Ok((
                CString::new(*name).map_err(|e| e.to_string())?,
                path_to_cstring(path)?,
            ))
        })
        .collect::<Result<Vec<(CString, CString)>, String>>()?;
    let raw_subtemplates: Vec<RawSubtemplate> = subtemplate_cstrings
        .iter()
        .map(|(name, path)| RawSubtemplate {
            name: name.as_ptr(),
            path: path.as_ptr(),
        })
        .collect();

    let raw = unsafe {
        cricket_generate_robot_source(
            urdf.as_ptr(),
            srdf.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
            end_effector
                .as_ref()
                .map_or(std::ptr::null(), |s| s.as_ptr()),
            template_path
                .as_ref()
                .map_or(std::ptr::null(), |s| s.as_ptr()),
            raw_subtemplates.as_ptr(),
            raw_subtemplates.len(),
            language.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
            opts.bounds
                .as_ref()
                .map_or(std::ptr::null(), |b| b as *const Bounds),
            extra_data_json
                .as_ref()
                .map_or(std::ptr::null(), |s| s.as_ptr()),
        )
    };

    // SAFETY: `raw` was just returned by cricket_generate_robot_source, whose `success` tag
    // tells us which union member it populated, and we free it exactly once regardless of
    // which branch we take below.
    let result = unsafe {
        if raw.success == 0 {
            let message = CStr::from_ptr(raw.data.error_message)
                .to_string_lossy()
                .into_owned();
            Err(message)
        } else {
            let success = raw.data.success;
            Ok(GenResult {
                source: CStr::from_ptr(success.source)
                    .to_string_lossy()
                    .into_owned(),
                data_json: CStr::from_ptr(success.data_json)
                    .to_string_lossy()
                    .into_owned(),
                robot_name: CStr::from_ptr(success.robot_name)
                    .to_string_lossy()
                    .into_owned(),
                dimension: success.dimension,
                n_spheres: success.n_spheres,
            })
        }
    };
    let mut raw = raw;
    unsafe { cricket_free_gen_result(&mut raw) };
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generates_panda_fk_from_embedded_template() {
        let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let resources = manifest_dir.join("vendor/cricket/resources");
        let result = generate_robot_source(
            &resources.join("panda/panda_spherized.urdf"),
            &GenOptions {
                srdf: Some(&resources.join("panda/panda.srdf")),
                end_effector: Some("panda_grasptarget"),
                extra_data_json: Some(r#"{"name": "Panda", "resolution": 32}"#),
                ..Default::default()
            },
        )
        .expect("generate_robot_source failed");

        assert_eq!(result.robot_name, "Panda");
        assert!(result.n_spheres > 0);
        assert!(result.source.contains("namespace"));
    }

    #[test]
    fn reports_missing_urdf_as_an_error() {
        let err =
            generate_robot_source(Path::new("/nonexistent/robot.urdf"), &GenOptions::default())
                .expect_err("expected an error for a missing URDF");
        assert!(!err.is_empty());
    }
}
