use std::sync::LazyLock;

use serde::Deserialize;

use crate::config::toml::TOML_STR;

mod toml;
pub mod updater;

#[derive(Clone, Debug, Deserialize)]
#[serde(from = "toml::Config")]
pub struct Config {
    pub fov: f32,

    pub angle_limit: [f32; 2],

    pub extra_player_height: f32,

    pub start_in_first_person: bool,

    pub show_tutorial: bool,

    pub soft_lock_on: bool,

    pub prioritize_lock_on: bool,

    pub unlocked_movement: bool,

    pub unobtrusive_dodges: bool,

    pub track_dodges: bool,

    pub track_damage: bool,

    pub restricted_sprint: bool,

    pub use_stabilizer: bool,

    pub stabilizer_window: f32,

    pub stabilizer_factor: f32,

    pub attach_dummy_id: u32,

    pub wobble: f32,

    pub rotation_lock_sp_effects: Vec<i32>,

    pub log_sp_effects: bool,

    /// (yaw, pitch) in radians around the aim direction in which the locked camera tracks.
    pub rotation_lock_aim_range: Option<(f32, f32)>,

    /// (yaw, pitch) in radians around a locked on target in which the locked camera tracks.
    pub rotation_lock_target_range: Option<(f32, f32)>,

    /// Time in seconds the camera takes to leave or rejoin the tracking range.
    pub rotation_lock_range_blend: f32,

    pub crosshair: CrosshairKind,

    pub crosshair_scale: (f32, f32),

    pub use_fov_correction: bool,

    pub use_barrel_correction: bool,

    pub correction_strength: f32,

    pub correction_cylindricity: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CrosshairKind {
    None,
    Cross,
    Dot,
    Circle,
    CircleDot,
    Angled,
}

impl From<toml::Config> for Config {
    fn from(config: toml::Config) -> Self {
        let degrees = config.fov.horizontal_fov.clamp(45.0, 130.0);
        let fov = degrees.to_radians();

        let mut extra_player_height = config.player.height_multiplier.clamp(0.975, 1.05) - 1.0;
        if extra_player_height > 0.0 {
            extra_player_height *= 0.5;
        }

        let stabilizer_window = config.stabilizer.smoothing_window.clamp(0.1, 1.0);
        let stabilizer_factor = config.stabilizer.smoothing_factor.clamp(0.0, 1.0);

        let attach_dummy_id = config.camera.attach_dummy_id;
        let wobble = config.camera.wobble.clamp(0.0, 3.0);

        // NaN safe: f32::max/min return the non-NaN argument.
        let rotation_range = |enabled: bool, x: f32, y: f32| {
            enabled.then(|| {
                (
                    x.max(0.0).min(180.0).to_radians(),
                    y.max(0.0).min(90.0).to_radians(),
                )
            })
        };

        let rotation_lock_aim_range = rotation_range(
            config.camera.rotation_lock_aim_range,
            config.camera.rotation_lock_aim_range_x,
            config.camera.rotation_lock_aim_range_y,
        );
        let rotation_lock_target_range = rotation_range(
            config.camera.rotation_lock_target_range,
            config.camera.rotation_lock_target_range_x,
            config.camera.rotation_lock_target_range_y,
        );

        let crosshair_scale_x = config.crosshair.scale_x.clamp(0.1, 4.0);
        let crosshair_scale_y = config.crosshair.scale_y.clamp(0.1, 4.0);

        let correction_strength = config.fov.fov_correction_strength.clamp(0.0, 1.0);
        let correction_cylindricity =
            config.fov.fov_correction_cylindricity.clamp(0.0, 1.0) * 1.5 + 0.5;

        let (use_fov_correction, use_barrel_correction) = match config.fov.fov_correction {
            toml::FovCorrection::None => (false, false),
            toml::FovCorrection::Fisheye => (true, false),
            toml::FovCorrection::Barrel => (true, true),
        };

        Self {
            fov,
            angle_limit: const { [f32::to_radians(-80.0), f32::to_radians(70.0)] },
            extra_player_height,
            start_in_first_person: config.gameplay.start_in_first_person,
            show_tutorial: config.gameplay.show_tutorial,
            prioritize_lock_on: config.gameplay.prioritize_lock_on,
            soft_lock_on: config.gameplay.soft_lock_on,
            unlocked_movement: config.gameplay.unlocked_movement,
            unobtrusive_dodges: config.gameplay.unobtrusive_dodges,
            track_dodges: config.gameplay.track_dodges,
            track_damage: config.gameplay.track_damage,
            restricted_sprint: config.gameplay.restricted_sprint,
            use_stabilizer: config.stabilizer.enabled,
            stabilizer_window,
            stabilizer_factor,
            attach_dummy_id,
            wobble,
            rotation_lock_sp_effects: config.camera.rotation_lock_sp_effects,
            log_sp_effects: config.camera.log_sp_effects,
            rotation_lock_aim_range,
            rotation_lock_target_range,
            rotation_lock_range_blend: config.camera.rotation_lock_range_blend.max(0.0).min(1.0),
            crosshair: config.crosshair.kind,
            crosshair_scale: (crosshair_scale_x, crosshair_scale_y),
            use_fov_correction,
            use_barrel_correction,
            correction_strength,
            correction_cylindricity,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        static DEFAULT: LazyLock<Config> = LazyLock::new(|| ::toml::from_str(TOML_STR).unwrap());
        DEFAULT.clone()
    }
}
