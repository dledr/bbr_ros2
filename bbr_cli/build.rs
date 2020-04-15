extern crate glob;
extern crate protoc_rust;

use std::fs;
use std::io::Write;

fn path_to_mod_proto(filename: &String, path: &str) -> String {
    filename
        .replace(&[&path, "/"].concat(), "pub mod ")
        .replace(".proto", ";\n")
}

fn path_to_mod(filename: &String, path: &str) -> String {
    filename.replace(&[&path, "/"].concat(), "pub mod ") + ";\n"
}

fn glob_simple(pattern: &str) -> Vec<String> {
    glob::glob(pattern)
        .expect("glob")
        .map(|g| {
            g.expect("item")
                .as_path()
                .to_str()
                .expect("utf-8")
                .to_owned()
        })
        .collect()
}

fn run_protoc_rust(path: &String) {
    let pattern = [&path, "/*.proto"].concat();
    let out_dir = ["src/", &path].concat();

    let proto_src_files = glob_simple(&pattern);
    println!("{:?}", proto_src_files);
    fs::create_dir_all(&out_dir).unwrap();

    protoc_rust::Codegen::new()
        .out_dir(&out_dir)
        .inputs(&proto_src_files
            .iter()
            .map(|a| a.as_ref())
            .collect::<Vec<&str>>())
        .include(&path)
        .run()
        .expect("unable to run protoc");

    let mod_rs = [&out_dir, "/mod.rs"].concat();
    let mut file = fs::File::create(&mod_rs).unwrap();
    for filename in proto_src_files.iter() {
        file.write_all(path_to_mod_proto(&filename, &path).as_bytes())
            .unwrap();
    }
}

fn main() {
    let path = "proto";
    let pattern = [&path, "/*/"].concat();
    let out_dir = ["src/", &path].concat();

    let proto_src_paths = glob_simple(&pattern);
    for proto_src_path in &proto_src_paths {
        run_protoc_rust(&proto_src_path);
    }

    let mod_rs = [&out_dir, "/mod.rs"].concat();
    let mut file = fs::File::create(&mod_rs).unwrap();
    for filename in &proto_src_paths {
        file.write_all(path_to_mod(&filename, &path).as_bytes())
            .unwrap();
    }
}
