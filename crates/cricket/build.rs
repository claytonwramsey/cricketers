fn main() {
    // `cargo:rustc-link-arg` from a dependency's build script (cricket-sys emits this same flag
    // for its own targets) doesn't propagate to a downstream crate's own binaries/tests -- this
    // crate is the one doing the final link now, so it needs its own copy. See
    // `cricket-sys/build.rs::emit_link_directives` for why: rustc defaults to lld here, which
    // rejects the `.debug_gdb_scripts` section GCC emits into the vendored static libs in debug
    // builds ("string is not null terminated"); ld.bfd and gold both accept it fine.
    println!("cargo:rustc-link-arg=-fuse-ld=bfd");
}
