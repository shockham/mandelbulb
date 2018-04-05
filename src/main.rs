extern crate caper;

use caper::utils::demo;
use std::str;

fn main() {
    let frag_str = str::from_utf8(include_bytes!("frag.glsl")).unwrap();
    demo(frag_str);
}
