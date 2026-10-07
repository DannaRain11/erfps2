use fromsoftware_shared::F32ModelMatrix;
use std::f32::consts::{PI, TAU};

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
    yaw_unwrapped: f32,
    last_yaw: f32,
}

pub struct Args {
    pub model_matrix: F32ModelMatrix,
    pub head_matrix: F32ModelMatrix,
    pub stabilizer_factor: f32,
    pub use_stabilizer: bool,
    pub wobble: f32,
    pub is_tracked: bool,
    pub lock_rotation: bool,
    pub rotation_limit: Option<(f32, f32)>,
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

    /// Follows the attachment point's rotation exactly, without easing, optionally keeping the
    /// view within `limit` (yaw, pitch in radians) of the direction the camera is aimed in.
    fn lock_rotation(&mut self, limit: Option<(f32, f32)>) {
        let Some((max_yaw, max_pitch)) = limit else {
            self.rotation = self.rotation_target;
            return;
        };

        let (yaw, pitch, roll) = self.rotation_target.to_euler(EulerRot::YXZ);

        // Track the yaw as a continuous angle, so a full turn is not mistaken for no turn.
        if self.was_locked {
            self.yaw_unwrapped += wrap_angle(yaw - self.last_yaw);
        } else {
            self.yaw_unwrapped = yaw;
        }
        self.last_yaw = yaw;

        self.rotation = Quat::from_euler(
            EulerRot::YXZ,
            self.yaw_unwrapped.clamp(-max_yaw, max_yaw),
            pitch.clamp(-max_pitch, max_pitch),
            roll,
        );
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
            self.lock_rotation(args.rotation_limit);
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

        // With the hard lock on the camera is aimed at the target, so the angles are relative to it.
        let rotation_limit = if context.lock_tgt.is_locked_on && !context.config.soft_lock_on {
            context.config.rotation_lock_target_limit
        } else {
            context.config.rotation_lock_aim_limit
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
            rotation_limit,
        }
    }
}

/// Wraps an angle in radians to the range `[-PI, PI)`.
fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TAU) - PI
}

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
