use std::{fmt, path::PathBuf};

/// Everything that can go wrong calling into cricket: a C++ exception propagated across the FFI
/// boundary (bad URDF/SRDF, an unsupported trace language, a malformed template, ...), or a path
/// that can't be handed across the boundary at all because it isn't valid UTF-8.
#[derive(Debug)]
pub enum Error {
    Cricket(String),
    NonUtf8Path(PathBuf),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Cricket(message) => f.write_str(message),
            Error::NonUtf8Path(path) => write!(f, "path {} is not valid UTF-8", path.display()),
        }
    }
}

impl std::error::Error for Error {}

impl From<cxx::Exception> for Error {
    fn from(e: cxx::Exception) -> Self {
        Error::Cricket(e.what().to_owned())
    }
}

pub type Result<T> = std::result::Result<T, Error>;

pub(crate) fn path_to_str(path: &std::path::Path) -> Result<&str> {
    path.to_str()
        .ok_or_else(|| Error::NonUtf8Path(path.to_owned()))
}
