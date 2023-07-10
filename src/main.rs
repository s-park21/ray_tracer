#[macro_use]
extern crate nalgebra as na;
use na::{vector, DMatrix, MatrixCross, Point3, ReshapableStorage, SMatrix, Vector2, Vector3};
mod ray_trace;
use std::{path::Path, thread};

// Run cargo doc --open to open the docs for this project
// Camera coordinate frame:
// Z is bore to camera
// X is right
// Y is down
// Make a spiderman hand sign

// Refer to "A minimal ray-tracer rendering simple shapes"

const h: usize = 1005; // Must be uneven
const w: usize = 1005; // Must be uneven
const PIXEL_HEIGHT: u32 = h as u32;
const PIXEL_WIDTH: u32 = w as u32;

fn main() {
    const width_mm: f32 = 10.0;
    const height_mm: f32 = 10.0;
    const focal_length: f32 = 10.0;
    let pixel_arr = DMatrix::zeros(h, w);
    let ray_vec: Vec<ray_trace::Ray> = vec![];

    let sensor = ray_trace::Sensor::new(width_mm, height_mm, PIXEL_WIDTH, PIXEL_HEIGHT, pixel_arr);

    let mut camera = ray_trace::Camera {
        focal_length_mm: focal_length,
        img_sensor: sensor,
        pin_hole: Vector3::new(0.0, 0.0, focal_length),
        ray_vec,
        zero_px_threshold: 0,
    };

    let light_source_origin = Point3::new(0.0, -10.0, -10.0);
    let light_source = ray_trace::LightSource {
        origin: light_source_origin,
        radius: 10.0,
    };

    let sphere = ray_trace::Sphere {
        origin: Point3::new(0.0, 0.0, 50.0),
        radius: 0.0,
        diffusivity: 0.001,
        scatter: 0.1;
    };
    let object = vec![sphere];
    let boundary = ray_trace::Cube::new(200.0, 0.5, 0.5);

    let ray_tracing_thread = thread::Builder::new()
        .spawn(move || {
            let mut ray_vector: Vec<ray_trace::Ray> = camera.spawn_rays();
            while !ray_vector.is_empty() {
                println!("{} ray vectors remaining", ray_vector.len());
                camera.draw_rays(&light_source, &object, &boundary, &mut ray_vector);
            }
            // Display results
            let mut byte_array = [0u8; (PIXEL_WIDTH * PIXEL_HEIGHT) as usize];

            for (index, element) in camera.img_sensor.pixel_arr.iter().enumerate() {
                byte_array[index] = *element;
            }
            let res = image::save_buffer(
                Path::new("image.png"),
                &byte_array,
                camera.img_sensor.pixel_width,
                camera.img_sensor.pixel_height,
                image::ColorType::L8,
            );
        })
        .unwrap();
    let res = ray_tracing_thread.join();
}
