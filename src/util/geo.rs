/// # Azimuthal Equidistant Projection
///
/// Formulas according to:
/// [Wolfram Alpha](http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html)
pub struct MapProjection {
    ref_lon: f64,
    ref_sin_lat: f64,
    ref_cos_lat: f64,
}

impl MapProjection {
    // Constants
    pub const CONSTANTS_ONE_G: f32 = 9.80665; // m/s^2
    pub const CONSTANTS_RADIUS_OF_EARTH: f64 = 6371000.0; // meters (m)
    pub const CONSTANTS_RADIUS_OF_EARTH_F: f32 = Self::CONSTANTS_RADIUS_OF_EARTH as f32; // meters (m)
    pub const CONSTANTS_EARTH_SPIN_RATE: f32 = 7.2921150e-5; // radians/second (rad/s)

    /// Constructor to initialize the MapProjection struct
    pub fn new(lat_0: f64, lon_0: f64) -> Self {
        let ref_lat = lat_0.to_radians();
        let ref_lon = lon_0.to_radians();
        let ref_sin_lat = ref_lat.sin();
        let ref_cos_lat = ref_lat.cos();

        Self {
            ref_lon,
            ref_sin_lat,
            ref_cos_lat,
        }
    }

    /// Project method
    pub fn project(&self, lat: f64, lon: f64) -> (f32, f32) {
        let lat_rad = lat.to_radians();
        let lon_rad = lon.to_radians();

        let sin_lat = lat_rad.sin();
        let cos_lat = lat_rad.cos();

        let cos_d_lon = (lon_rad - self.ref_lon).cos();

        let arg =
            (self.ref_sin_lat * sin_lat + self.ref_cos_lat * cos_lat * cos_d_lon).clamp(-1.0, 1.0);
        let c = arg.acos();

        let mut k = 1.0;

        if c.abs() > 0.0 {
            k = c / c.sin();
        }

        let x = (k
            * (self.ref_cos_lat * sin_lat - self.ref_sin_lat * cos_lat * cos_d_lon)
            * Self::CONSTANTS_RADIUS_OF_EARTH) as f32;
        let y =
            (k * cos_lat * (lon_rad - self.ref_lon).sin() * Self::CONSTANTS_RADIUS_OF_EARTH) as f32;

        (x, y)
    }
}
