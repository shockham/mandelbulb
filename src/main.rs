extern crate caper;

use caper::utils::demo;
use std::str;

fn main() {
    let shader = include_bytes!("frag.glsl");
    let frag_str = str::from_utf8(shader).unwrap();
    demo(frag_str);
}
