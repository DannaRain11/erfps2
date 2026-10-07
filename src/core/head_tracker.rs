use fromsoftware_shared::F32ModelMatrix;
use glam::{EulerRot, Mat4, Quat, Vec3};

use crate::{
    core::{
        BehaviorState, CoreLogicContext, frame_cached::FrameCache, stabilizer::CameraStabilizer,
        world::World,
    },
    player::PlayerExt,
};

#[derive(Default)]
pub struct HeadTracker {
    last: Option<Quat>,
    rotation: Quat,
    rotation_target: Quat,
    stabilizer: CameraStabilizer,
    output: Option<Output>,
    was_locked: bool,
    engaged: bool,
    blend: f32,
    anchor: Quat,
}

pub struct Args {
    pub model_matrix: F32ModelMatrix,
    pub head_matrix: F32ModelMatrix,
    pub stabilizer_factor: f32,
    pub use_stabilizer: bool,
    pub wobble: f32,
    pub is_tracked: bool,
    pub lock_rotation: bool,
    pub rotation_range: Option<(f32, f32)>,
    pub range_blend: f32,
}

pub struct Output {
    pub tracking_rotation: Quat,
    pub stabilized_head_position: Vec3,
    pub head_matrix: F32ModelMatrix,
}

impl HeadTracker {
    pub fn set_stabilizer_window(&mut self, window: f32) {
        self.stabilizer.set_window(window);
    }

    /// Follows the attachment point's rotation exactly, without easing.
    ///
    /// With a `range` (yaw, pitch in radians) the camera only follows while the attachment point
    /// has turned no further than the range from where it started. Outside of it the camera
    /// returns to the fixed direction it is aimed in, and follows again once the attachment
    /// point is back within the range.
    fn lock_rotation(&mut self, frame_time: f32, range: Option<(f32, f32)>, blend_time: f32) {
        let Some((max_yaw, max_pitch)) = range else {
            self.engaged = true;
            self.blend = 1.0;
            self.anchor = self.rotation_target;
            self.rotation = self.rotation_target;
            return;
        };

        if !self.was_locked {
            self.engaged = true;
            self.blend = 1.0;
            self.anchor = self.rotation_target;
        }

        let (yaw, pitch, _) = self.rotation_target.to_euler(EulerRot::YXZ);

        // Rejoining the range requires being slightly inside of it, which stops the camera
        // from flickering when the attachment point hovers on its edge.
        let margin = if self.engaged { 0.0 } else { RANGE_HYSTERESIS };

        let in_range = yaw.abs() <= (max_yaw - margin).max(0.0)
            && pitch.abs() <= (max_pitch - margin).max(0.0);

        self.engaged = in_range;

        // While leaving the range the camera blends out of the last rotation that was in range.
        if in_range {
            self.anchor = self.rotation_target;
        }

        let step = if blend_time > 0.0 {
            frame_time / blend_time
        } else {
            1.0
        };

        self.blend = if in_range {
            (self.blend + step).min(1.0)
        } else {
            (self.blend - step).max(0.0)
        };

        self.rotation = if self.blend >= 1.0 {
            self.anchor
        } else if self.blend <= 0.0 {
            Quat::IDENTITY
        } else {
            Quat::IDENTITY.slerp(self.anchor, self.blend)
        };
    }

    fn rotate_towards_target(&mut self, frame_time: f32) {
        let distance = self.rotation.angle_between(self.rotation_target);
        let step = rip(distance, 0.0, 1.0, frame_time);

        self.rotation = self.rotation.rotate_towards(self.rotation_target, step);
    }
}

impl FrameCache for HeadTracker {
    type Input = Args;
    type Output<'a> = &'a Output;

    fn update(&mut self, frame_time: f32, args: Self::Input) -> Self::Output<'_> {
        let head_position = args.head_matrix.translation();

        let player_matrix = Mat4::from(args.model_matrix);

        let raw_local_pos = player_matrix.inverse().project_point3(head_position);

        // Moving average of the attachment point relative to the player model.
        let average = self.stabilizer.update(frame_time, raw_local_pos);

        let mut local_head_pos = raw_local_pos;

        if args.use_stabilizer {
            let delta = average - raw_local_pos;

            local_head_pos += delta.clamp_length_max(args.stabilizer_factor * 0.1);
        }

        // Scale the translational movement around its average.
        // 1.0 leaves the position untouched, translation only: rotation is handled separately.
        local_head_pos = average + (local_head_pos - average) * args.wobble;

        let head_position = player_matrix.project_point3(local_head_pos);

        let input = Quat::from_mat3a(&args.head_matrix.rotation());

        if args.is_tracked
            && let Some(last) = self.last
        {
            self.rotation_target *= last.inverse() * input;
            self.rotation_target = self.rotation_target.normalize();
        } else {
            self.rotation_target = Quat::IDENTITY;
        }

        self.last = Some(input);

        if args.lock_rotation {
            self.lock_rotation(frame_time, args.rotation_range, args.range_blend);
        } else {
            self.rotate_towards_target(frame_time);
        }

        self.was_locked = args.lock_rotation;

        self.output.insert(Output {
            tracking_rotation: self.rotation,
            stabilized_head_position: head_position,
            head_matrix: args.head_matrix,
        })
    }

    fn get_cached(&mut self, _frame_time: f32, _input: Self::Input) -> Self::Output<'_> {
        self.output.as_ref().expect("FrameCache logic error")
    }

    fn reset(&mut self) {
        self.stabilizer.reset();
        self.last = None;
        self.was_locked = false;
    }
}

impl From<&CoreLogicContext<'_, World<'_>>> for Args {
    fn from(context: &CoreLogicContext<'_, World<'_>>) -> Self {
        let head_matrix = context.player.head_matrix(context.config.attach_dummy_id);
        let model_matrix = context.player.model_matrix();

        if context.config.log_sp_effects {
            context.player.log_sp_effect_changes();
        }

        let lock_rotation = context
            .player
            .has_any_sp_effect(&context.config.rotation_lock_sp_effects);

        // With the hard lock on the camera is aimed at the target, so the range is relative to it.
        let rotation_range = if context.lock_tgt.is_locked_on && !context.config.soft_lock_on {
            context.config.rotation_lock_target_range
        } else {
            context.config.rotation_lock_aim_range
        };

        let is_tracked = lock_rotation
            || context.player.is_in_throw()
            || (context.config.track_damage && context.has_state(BehaviorState::Damage))
            || (context.config.track_dodges && context.has_state(BehaviorState::Evasion));

        Self {
            head_matrix,
            model_matrix,
            stabilizer_factor: context.config.stabilizer_factor,
            use_stabilizer: context.config.use_stabilizer,
            wobble: context.config.wobble,
            is_tracked,
            lock_rotation,
            rotation_range,
            range_blend: context.config.rotation_lock_range_blend,
        }
    }
}

/// How far (in radians) inside of the tracking range the camera has to be to rejoin it.
const RANGE_HYSTERESIS: f32 = 2.0 * std::f32::consts::PI / 180.0;

/**
    Computes a signed distance step that moves `distance` toward 0 over the next `timedelta`.

      Curve: d(t) = (t * b)^6 - a
    Inverse: t(d) = (d + a)^(1/6) / b
       Step:        d(t) - d(t-Δt)

    Method:
    - Interpret `distance` as the remaining distance to zero, offset by `curve_offset`.
    - Convert remaining distance -> remaining time using t(d), scaled by `curve_scale`.
    - Advance time by `timedelta` and map back using d(t) to get the new remaining distance.
    - Return step = distance - distance_new, clamped to \[0, distance\].
*/
fn rip(distance: f32, curve_offset: f32, curve_scale: f32, timedelta: f32) -> f32 {
    let sign = distance.signum();
    let distance = distance.abs();

    let time_remaining = (distance + curve_offset).powf(1.0 / 6.0) / curve_scale;
    let time_new = (time_remaining - timedelta).max(0.0);

    let distance_new = (time_new * curve_scale).powi(6) - curve_offset;

    let step = (distance - distance_new).max(0.0).min(distance);

    step * sign
}
