extern crate nalgebra as na;
use na::{DMatrix, Point3, Vector2, Vector3};

enum intersection {
    object,
    boundary,
    light_source,
    error,
}

pub struct Ray {
    vec: Vector3<f32>,
    point: Point3<f32>,
    pixel_coord: Vector2<u32>,
    current_obj_idx: usize, // Index of the object that this ray is emitted from
}

pub struct Sensor {
    pub width_mm: f32,
    pub height_mm: f32,
    pub pixel_width: u32,  // Number of pixels wide
    pub pixel_height: u32, // Number of pixels high
    pub pixel_width_mm: f32,
    pub pixel_height_mm: f32,
    pub pixel_arr: DMatrix<u8>,
}

impl Sensor {
    pub fn new(
        width_mm: f32,
        height_mm: f32,
        pixel_width: u32,
        pixel_height: u32,
        pixel_arr: DMatrix<u8>,
    ) -> Self {
        let mut pixel_array = pixel_arr;
        pixel_array.fill(255);
        let pixel_width_mm: f32 = width_mm / pixel_width as f32;
        let pixel_height_mm: f32 = height_mm / pixel_height as f32;

        Self {
            width_mm,
            height_mm,
            pixel_width,
            pixel_height,
            pixel_width_mm,
            pixel_height_mm,
            pixel_arr: pixel_array,
        }
    }
}

pub struct Camera {
    pub focal_length_mm: f32,
    pub img_sensor: Sensor,
    pub pin_hole: Vector3<f32>, // Position of pin hole in camera coordinates
    pub ray_vec: Vec<Ray>,
    pub zero_px_threshold: u8,
}

impl Camera {
    pub fn new(
        focal_length_mm: f32,
        img_sensor: Sensor,
        ray_vec: Vec<Ray>,
        zero_px_threshold: u8,
    ) -> Self {
        let pin_hole = Vector3::new(0.0, 0.0, focal_length_mm);
        Self {
            focal_length_mm,
            img_sensor,
            pin_hole,
            ray_vec,
            zero_px_threshold,
        }
    }
    fn randomise_normal_vec(vec_norm: Vector3<f32>, scatter_probability: f32) -> Vector3<f32> {
        let rand0 = 0.1;
        let rand1 = 0.2;
        let rand2 = 0.3;
        let random_number = 0.9;
        let rand_vec: Vector3<f32> = Vector3::new(rand0, rand1, rand2);
        let norm_vec_rand: Vector3<f32> = vec_norm;
        if random_number >= scatter_probability {
            norm_vec_rand = vec_norm * rand_vec;
        }
        norm_vec_rand.normalize()
    }
    fn check_face_intersection(
        &mut self,
        ray: &Ray,
        boundary: &Cube,
        p_on_plane1: Point3<f32>,
        p_on_plane2: Point3<f32>,
        p_on_plane3: Point3<f32>,
    ) -> (f32, Vector3<f32>) {
        let vec1: Vector3<f32> = p_on_plane1 - p_on_plane2;
        let vec2: Vector3<f32> = p_on_plane2 - p_on_plane3;
        let vec_norm = vec1.cross(&vec2).normalize(); // Normal vec to plane
        let norm_point: Point3<f32> = (-boundary.side_length / 2.0 * vec_norm).into(); // Normal vec intersection point with plane
        let denom = ray.vec.dot(&vec_norm);
        if denom.abs() >= 1e-6 {
            ((norm_point - ray.point).dot(&vec_norm) / denom, vec_norm)
        } else {
            (0.0, Vector3::new(0.0, 0.0, 0.0))
        }
    }
    fn check_intersection(
        &mut self,
        ray: &Ray,
        object: &[Sphere],
        boundary: &Cube,
        light_source: &LightSource,
    ) -> (intersection, Point3<f32>, usize, Vector3<f32>) {
        // Check intersection with objects
        for (obj_idx, obj) in object.iter().enumerate() {
            // Refer to:
            // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html
            /* Check intersection with the sphere */
            // Find vector between origin of ray and origin of sphere
            if ray.current_obj_idx != obj_idx {
                let L = obj.origin - ray.point;
                let Tca = L.dot(&ray.vec);
                if Tca >= 0.0 {
                    let d = (L.dot(&L) - Tca.powi(2)).sqrt();
                    if d >= 0.0 && d <= obj.radius {
                        // Ray intersects sphere
                        let Thc = (obj.radius.powi(2) - d.powi(2)).sqrt();
                        let t0 = Tca - Thc;
                        let t1 = Tca + Thc;
                        let P1 = ray.point + t0 * ray.vec.normalize();
                        let P2 = ray.point + t1 * ray.vec.normalize();
                        if (P1 - Point3::new(0.0, 0.0, 0.0)).norm()
                            < (P2 - Point3::new(0.0, 0.0, 0.0)).norm()
                        {
                            let norm_vec_rand =
                                randomise_normal_vec(P1 - object[obj_idx].origin, obj.scatter);
                            return (intersection::object, P1, obj_idx, norm_vec_rand);
                        } else {
                            let norm_vec_rand =
                                randomise_normal_vec(P2 - object[obj_idx].origin, obj.scatter);
                            return (intersection::object, P2, obj_idx, norm_vec_rand);
                        }
                    }
                }
            }
        }
        // Check intersection with light source
        let L = light_source.origin - ray.point;
        let Tca = L.dot(&ray.vec.normalize());
        if Tca >= 0.0 {
            let d = (L.dot(&L) - Tca.powi(2)).sqrt();
            if d >= 0.0 && d <= light_source.radius {
                // Ray intersects sphere
                let Thc = (light_source.radius.powi(2) - d.powi(2)).sqrt();
                let t0 = Tca - Thc;
                let t1 = Tca + Thc;
                let P1 = ray.point + t0 * ray.vec.normalize();
                let P2 = ray.point + t1 * ray.vec.normalize();

                if (P1 - Point3::new(0.0, 0.0, 0.0)).norm()
                    < (P2 - Point3::new(0.0, 0.0, 0.0)).norm()
                {
                    return (
                        intersection::light_source,
                        P1,
                        0,
                        (P1 - light_source.origin).normalize(),
                    );
                } else {
                    return (
                        intersection::light_source,
                        P2,
                        0,
                        (P2 - light_source.origin).normalize(),
                    );
                }
            }
        }
        let mut vec_norm: Vector3<f32>;
        let mut t0: f32;
        let mut t1: f32;
        let mut t2: f32;
        let mut t3: f32;
        let mut t4: f32;
        let mut t5: f32;
        // Check intersection with boundaries
        // Go through all six faces of the bounding box
        // Back face
        (t0, vec_norm) = self.check_face_intersection(
            ray,
            boundary,
            boundary.vertices[0],
            boundary.vertices[1],
            boundary.vertices[2],
        );
        if t0 > 0.0 {
            let norm_vec_rand = randomise_normal_vec(vec_norm, boundary.scatter);
            let intersection_point = ray.point + t0 * ray.vec.normalize();
            return (intersection::boundary, intersection_point, 0, norm_vec_rand);
        }
        // Front face
        (t1, vec_norm) = self.check_face_intersection(
            ray,
            boundary,
            boundary.vertices[6],
            boundary.vertices[5],
            boundary.vertices[4],
        );
        if t1 > 0.0 {
            let norm_vec_rand = randomise_normal_vec(vec_norm, boundary.scatter);
            let intersection_point = ray.point + t1 * ray.vec.normalize();
            return (intersection::boundary, intersection_point, 0, norm_vec_rand);
        }
        // Right face
        (t2, vec_norm) = self.check_face_intersection(
            ray,
            boundary,
            boundary.vertices[2],
            boundary.vertices[1],
            boundary.vertices[5],
        );
        if t2 > 0.0 {
            let norm_vec_rand = randomise_normal_vec(vec_norm, boundary.scatter);
            let intersection_point = ray.point + t2 * ray.vec.normalize();
            return (intersection::boundary, intersection_point, 0, norm_vec_rand);
        }
        // Left face
        (t3, vec_norm) = self.check_face_intersection(
            ray,
            boundary,
            boundary.vertices[4],
            boundary.vertices[0],
            boundary.vertices[3],
        );
        if t3 > 0.0 {
            let norm_vec_rand = randomise_normal_vec(vec_norm, boundary.scatter);
            let intersection_point = ray.point + t3 * ray.vec.normalize();
            return (intersection::boundary, intersection_point, 0, norm_vec_rand);
        }
        // Top face
        (t4, vec_norm) = self.check_face_intersection(
            ray,
            boundary,
            boundary.vertices[0],
            boundary.vertices[1],
            boundary.vertices[4],
        );
        if t4 > 0.0 {
            let norm_vec_rand = randomise_normal_vec(vec_norm, boundary.scatter);
            let intersection_point = ray.point + t4 * ray.vec.normalize();
            return (intersection::boundary, intersection_point, 0, norm_vec_rand);
        }
        // Bottom face
        (t5, vec_norm) = self.check_face_intersection(
            ray,
            boundary,
            boundary.vertices[2],
            boundary.vertices[3],
            boundary.vertices[7],
        );
        if t5 > 0.0 {
            let norm_vec_rand = randomise_normal_vec(vec_norm, boundary.scatter);
            let intersection_point = ray.point + t5 * ray.vec.normalize();
            return (intersection::boundary, intersection_point, 0, norm_vec_rand);
        }

        return (
            intersection::error,
            Point3::new(0.0, 0.0, 0.0),
            0,
            Vector3::new(0.0, 0.0, 0.0),
        );
    }
    pub fn spawn_rays(&mut self) -> Vec<Ray> {
        // Define ray vector
        let mut ray_vec: Vec<Ray> = vec![];
        for i in 0..self.img_sensor.pixel_height as usize {
            for j in 0..self.img_sensor.pixel_width as usize {
                let ray = Ray {
                    vec: (self.pin_hole
                        - Vector3::new(
                            (j as f32 - self.img_sensor.pixel_width as f32 / 2.0)
                                * self.img_sensor.pixel_width_mm
                                + self.img_sensor.pixel_width_mm / 2.0,
                            (i as f32 - self.img_sensor.pixel_height as f32 / 2.0)
                                * self.img_sensor.pixel_height_mm
                                + self.img_sensor.pixel_height_mm / 2.0,
                            0.0,
                        ))
                    .normalize(),
                    point: Point3::new(
                        (j as f32 - self.img_sensor.pixel_width as f32 / 2.0)
                            * self.img_sensor.pixel_width_mm
                            + self.img_sensor.pixel_width_mm / 2.0,
                        (i as f32 - self.img_sensor.pixel_height as f32 / 2.0)
                            * self.img_sensor.pixel_height_mm
                            + self.img_sensor.pixel_height_mm / 2.0,
                        0.0,
                    ),
                    pixel_coord: Vector2::new(i as u32, j as u32),
                    current_obj_idx: usize::MAX,
                };
                // Add ray to ray vector
                ray_vec.push(ray);
                // println!("(i,j): ({},{})\nRay: {:?}", i, j, ray_vec.last());
            }
        }
        ray_vec
    }
    fn decrement_pixel(&mut self, mut pixel: u8, diffusivity_factor: f32) -> u8 {
        pixel -= (pixel as f32 * diffusivity_factor).round() as u8;
        pixel
    }
    pub fn draw_rays(
        &mut self,
        light_source: &LightSource,
        object: &Vec<Sphere>,
        boundary: &Cube,
        ray_vec: &mut Vec<Ray>,
    ) {
        let mut kill_rays: Vec<usize> = vec![];
        // Loop through rays
        for (i, ray) in ray_vec.iter_mut().enumerate() {
            // Check intersection with object, bounary and light source
            let (result, intersection_point, obj_idx, vec_norm) =
                self.check_intersection(ray, object, boundary, light_source);
            match result {
                intersection::object => {
                    // println!("Object intersection");
                    // Intersection found, redefine rays to exist from the intersection point
                    // Define normal vector
                    let reflected_vec: Vector3<f32> =
                        ray.vec - 2.0 * (ray.vec.dot(&vec_norm)) * vec_norm;
                    // Assign ray's vector as new reflected vector
                    ray.vec = reflected_vec;
                    // Assign ray's point as point of intersection
                    ray.point = intersection_point;
                    // Reassign the current object's index
                    ray.current_obj_idx = obj_idx;
                    // Decrement the value of the ray's pixel by the diffusion factor
                    let px_val = self.img_sensor.pixel_arr
                        [(ray.pixel_coord.x as usize, ray.pixel_coord.y as usize)];

                    self.img_sensor.pixel_arr
                        [(ray.pixel_coord.x as usize, ray.pixel_coord.y as usize)] =
                        self.decrement_pixel(px_val, object[obj_idx].diffusivity);
                }
                intersection::boundary => {
                    // println!("Boundary intersection");
                    // Decrement the value of the ray's pixel by the diffusion factor
                    let px_val = self.img_sensor.pixel_arr
                        [(ray.pixel_coord.x as usize, ray.pixel_coord.y as usize)];

                    self.img_sensor.pixel_arr
                        [(ray.pixel_coord.x as usize, ray.pixel_coord.y as usize)] =
                        self.decrement_pixel(px_val, boundary.diffusivity);

                    if self.img_sensor.pixel_arr
                        [(ray.pixel_coord.x as usize, ray.pixel_coord.y as usize)]
                        <= self.zero_px_threshold
                    {
                        // Set the pixel value to zero
                        self.img_sensor.pixel_arr
                            [(ray.pixel_coord.x as usize, ray.pixel_coord.y as usize)] = 0;
                        // Store index of ray to kill
                        kill_rays.push(i);
                    } else {
                        ray.point = intersection_point;
                        // let randomised_norm = vec_norm * rand();
                        let reflected_vec: Vector3<f32> =
                            ray.vec - 2.0 * (ray.vec.dot(&vec_norm)) * vec_norm;
                        ray.vec = reflected_vec;
                    }
                }
                intersection::light_source => {
                    // println!("Light source intersection");
                    // Kill the ray
                    kill_rays.push(i);
                }
                intersection::error => {
                    // println!("Error intersection");
                    // Set the pixel value to zero
                    self.img_sensor.pixel_arr
                        [(ray.pixel_coord.x as usize, ray.pixel_coord.y as usize)] = 0;
                    // Store index of ray to kill
                    kill_rays.push(i);
                }
            }
        }

        // Kill dead rays
        for &mut dead_ray_idx in kill_rays.iter_mut().rev() {
            // Kill each ray
            ray_vec.remove(dead_ray_idx);
        }
    }
}

pub struct Sphere {
    pub origin: Point3<f32>,
    pub radius: f32,
    pub diffusivity: f32,
    pub scatter: f32, // A number between 0 and 1
}

pub struct TriangleSurface {}

pub struct LightSource {
    pub origin: Point3<f32>,
    pub radius: f32,
    pub diffusivity: f32,
    pub scatter: f32, // A number between 0 and 1
}

// Cube is assumed to be aligned with the xyz axes
pub struct Cube {
    centre_point: Point3<f32>,
    side_length: f32,
    vertices: [Point3<f32>; 8],
    diffusivity: f32,
    scatter: f32, // A number between 0 and 1
}

impl Cube {
    pub fn new(side_length: f32, diffusivity: f32, scatter: f32) -> Self {
        let hsl: f32 = side_length / 2.0;
        let v0 = Point3::new(-hsl, -hsl, -hsl); // BTL - Back top left
        let v1 = Point3::new(hsl, -hsl, -hsl); // BTR
        let v2 = Point3::new(hsl, hsl, -hsl); // BBR - Back bottom right
        let v3 = Point3::new(-hsl, hsl, -hsl); // BBL
        let v4 = Point3::new(-hsl, -hsl, hsl); // FTL - Front top left
        let v5 = Point3::new(hsl, -hsl, hsl); // FTR
        let v6 = Point3::new(hsl, hsl, hsl); // FBR
        let v7 = Point3::new(-hsl, hsl, hsl); // FBL
        let vertices = [v0, v1, v2, v3, v4, v5, v6, v7];
        let centre_point = Point3::new(0.0, 0.0, 0.0);
        Self {
            centre_point,
            side_length,
            vertices,
            diffusivity,
            scatter,
        }
    }
}
