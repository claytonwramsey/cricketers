use std::{
    collections::{BTreeSet, HashMap},
    path::Path,
};

use crate::error::{Result, path_to_str};

/// A rigid transform: an SE(3) pose, decomposed as a translation and a row-major rotation
/// matrix (`rotation[row][col]`) since cricket-sys can't hand a `pinocchio::SE3` across FFI.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Isometry3 {
    pub translation: [f64; 3],
    pub rotation: [[f64; 3]; 3],
}

fn isometry_from_raw(translation: [f64; 3], rotation: [f64; 9]) -> Isometry3 {
    Isometry3 {
        translation,
        rotation: [
            [rotation[0], rotation[1], rotation[2]],
            [rotation[3], rotation[4], rotation[5]],
            [rotation[6], rotation[7], rotation[8]],
        ],
    }
}

/// Optional Cartesian bounds for FreeFlyer / Planar joints.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Bounds {
    pub lower: [f64; 3],
    pub upper: [f64; 3],
}

impl From<Bounds> for cricket_sys::Bounds {
    fn from(b: Bounds) -> Self {
        cricket_sys::Bounds {
            lower: b.lower,
            upper: b.upper,
        }
    }
}

/// One of the spheres cricket spherizes a robot's collision geometry into, positioned relative
/// to its parent joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sphere {
    pub geom_index: usize,
    pub radius: f32,
    pub parent_joint: usize,
    pub parent_frame: usize,
    pub pose: Isometry3,
}

impl From<cricket_sys::SphereInfo> for Sphere {
    fn from(s: cricket_sys::SphereInfo) -> Self {
        Sphere {
            geom_index: s.geom_index,
            radius: s.radius,
            parent_joint: s.parent_joint,
            parent_frame: s.parent_frame,
            pose: isometry_from_raw(s.translation, s.rotation),
        }
    }
}

/// The output language a `trace_*` function renders generated code in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Language {
    #[default]
    Cpp,
    Rust,
}

impl Language {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Language::Cpp => "c++",
            Language::Rust => "rust",
        }
    }
}

/// One fragment of traced, code-generated kinematics/collision code, along with the temporary
/// variable and output counts the template needs to declare buffers for it.
#[derive(Clone, Debug, PartialEq)]
pub struct Traced {
    pub code: String,
    pub temp_variables: usize,
    pub outputs: usize,
}

impl From<cricket_sys::Traced> for Traced {
    fn from(t: cricket_sys::Traced) -> Self {
        Traced {
            code: t.code,
            temp_variables: t.temp_variables,
            outputs: t.outputs,
        }
    }
}

/// A parsed robot: forward-kinematics model, spherized collision geometry, and self-collision
/// data, derived from a URDF (+ optional SRDF). Every accessor mirrors a field or method of
/// cricket's `RobotInfo`, translated into native Rust containers (`HashMap`, `BTreeSet`,
/// `Vec<Vec<_>>`) in place of cricket-sys's flattened FFI-safe shapes.
pub struct RobotInfo(cxx::UniquePtr<cricket_sys::RobotInfo>);

impl RobotInfo {
    /// Parses a URDF (+ optional SRDF) into a `RobotInfo`. With no SRDF, self-collisions are
    /// guessed by sampling random configurations rather than read from file. With no end
    /// effector, the URDF's most distal link is used.
    pub fn new(urdf: &Path, srdf: Option<&Path>, end_effector: Option<&str>) -> Result<Self> {
        let urdf = path_to_str(urdf)?;
        let srdf = srdf.map(path_to_str).transpose()?.unwrap_or("");
        let inner = cricket_sys::robot_info_new(urdf, srdf, end_effector.unwrap_or(""))?;
        Ok(Self(inner))
    }

    /// The configuration space dimension (`model.nq`).
    pub fn dimension(&self) -> usize {
        cricket_sys::dimension(&self.0)
    }

    pub fn n_spheres(&self) -> usize {
        cricket_sys::n_spheres(&self.0)
    }

    pub fn min_radius(&self) -> f32 {
        cricket_sys::min_radius(&self.0)
    }

    pub fn max_radius(&self) -> f32 {
        cricket_sys::max_radius(&self.0)
    }

    pub fn min_radius_mobile(&self) -> f32 {
        cricket_sys::min_radius_mobile(&self.0)
    }

    pub fn max_radius_mobile(&self) -> f32 {
        cricket_sys::max_radius_mobile(&self.0)
    }

    pub fn min_bounding_radius_mobile(&self) -> f32 {
        cricket_sys::min_bounding_radius_mobile(&self.0)
    }

    pub fn max_bounding_radius_mobile(&self) -> f32 {
        cricket_sys::max_bounding_radius_mobile(&self.0)
    }

    pub fn base_position(&self) -> [f32; 3] {
        cricket_sys::base_position(&self.0)
    }

    pub fn end_effector_name(&self) -> String {
        cricket_sys::end_effector_name(&self.0)
    }

    pub fn end_effector_index(&self) -> usize {
        cricket_sys::end_effector_index(&self.0)
    }

    /// The robot's metadata (bounds, joint topology, sphere/link indices, ...) that
    /// `generate_robot_source`'s template context is built from, as parsed JSON.
    pub fn metadata(&self, bounds: Option<Bounds>) -> Result<serde_json::Value> {
        let json = cricket_sys::robot_info_json(
            &self.0,
            bounds.is_some(),
            bounds.unwrap_or_default().into(),
        )?;
        Ok(serde_json::from_str(&json).expect("cricket::RobotInfo::json always returns an object"))
    }

    pub fn dof_to_joint_names(&self) -> Vec<String> {
        cricket_sys::dof_to_joint_names(&self.0)
    }

    pub fn spheres(&self) -> Vec<Sphere> {
        cricket_sys::spheres(&self.0)
            .into_iter()
            .map(Sphere::from)
            .collect()
    }

    /// Per-link bounding spheres, keyed by frame index.
    pub fn bounding_spheres(&self) -> HashMap<usize, Sphere> {
        cricket_sys::bounding_spheres(&self.0)
            .into_iter()
            .map(|entry| (entry.frame_index, Sphere::from(entry.sphere)))
            .collect()
    }

    /// Frame-index pairs allowed to collide (i.e. not filtered out as adjacent or
    /// always/never-colliding), matching `RobotInfo::allowed_link_pairs`.
    pub fn allowed_link_pairs(&self) -> BTreeSet<(usize, usize)> {
        cricket_sys::allowed_link_pairs(&self.0)
            .into_iter()
            .map(|pair| (pair.first, pair.second))
            .collect()
    }

    /// `per_link_spheres[link_index]` is the list of sphere geometry indices attached to that
    /// link.
    pub fn per_link_spheres(&self) -> Vec<Vec<usize>> {
        let mut raw = cricket_sys::per_link_spheres(&self.0);
        raw.sort_by_key(|entry| entry.link_index);
        raw.into_iter().map(|entry| entry.sphere_indices).collect()
    }

    pub fn links_with_geometry(&self) -> Vec<usize> {
        cricket_sys::links_with_geometry(&self.0)
    }

    pub fn bounding_sphere_index(&self) -> Vec<usize> {
        cricket_sys::bounding_sphere_index(&self.0)
    }

    /// Re-derives `allowed_link_pairs` by sampling `samples` random configurations and checking
    /// which link pairs actually collide, discarding whatever the constructor (or a previous
    /// call) computed. Useful for trading accuracy for speed, or vice versa, after construction.
    pub fn guess_self_collisions(&mut self, samples: usize) -> Result<()> {
        cricket_sys::guess_self_collisions(self.0.pin_mut(), samples)?;
        Ok(())
    }

    pub fn trace_sphere_cc_fk(
        &self,
        language: Language,
        spheres: bool,
        bounding_spheres: bool,
        fk: bool,
    ) -> Result<Traced> {
        Ok(cricket_sys::trace_sphere_cc_fk(
            &self.0,
            language.as_str(),
            spheres,
            bounding_spheres,
            fk,
        )?
        .into())
    }

    pub fn trace_map_to_configuration(
        &self,
        language: Language,
        bounds: Option<Bounds>,
    ) -> Result<Traced> {
        Ok(cricket_sys::trace_map_to_configuration(
            &self.0,
            language.as_str(),
            bounds.is_some(),
            bounds.unwrap_or_default().into(),
        )?
        .into())
    }

    pub fn trace_interpolate(&self, language: Language) -> Result<Traced> {
        Ok(cricket_sys::trace_interpolate(&self.0, language.as_str())?.into())
    }

    pub fn trace_interpolate_block(&self, language: Language) -> Result<Traced> {
        Ok(cricket_sys::trace_interpolate_block(&self.0, language.as_str())?.into())
    }

    pub fn trace_distance(&self, language: Language) -> Result<Traced> {
        Ok(cricket_sys::trace_distance(&self.0, language.as_str())?.into())
    }
}
