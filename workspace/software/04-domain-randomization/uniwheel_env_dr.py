"""
Gymnasium wrapper for MuJoCo simulator that loads and runs the environment for the uni wheel
reaction-wheel balancing robot. Adds domain randomization for bumps, pushes, IMU noise, motor
noise, mass/friction/motor-gain randomization, on top of the base UniWheelEnv from
workspace/software/02-balance-bot-with-ppo/uniwheel_env.py.

Two deliberate deviations from the reference repo's balance_bot_env_dr.py:

1. "Ridge torque" (random axle disturbance simulating a tire ridge hitting the ground) is only
   ever applied to the `wheel` DOF, never `reaction_wheel` - the reaction wheel is fully enclosed
   and never touches the ground (see docs/feasibility_analysis.md), so it has no tire to hit a
   ridge with. Applying it there anyway would be training against a disturbance that can't
   physically occur.
2. `yaw_penalty_coef` defaults to 0.0, not 0.1. The reference repo's bot is differential-drive -
   it can actually steer yaw with its two wheels, so penalizing unwanted yaw drift discourages a
   real, achievable behavior. Neither of our actuators (wheel_motor: axis Y, reaction_wheel_motor:
   axis X) has any torque component about the vertical (yaw) axis - any yaw rotation here is a
   passive side effect (ground-contact friction torsion, gyroscopic coupling between the two
   wheels), not something the policy can deliberately cause or correct. Penalizing an unactuatable
   variable gives no useful training signal - at best it's inert, at worst it's reward noise that
   could push the policy toward a worse pitch/roll trade-off chasing a yaw reduction it structurally
   cannot deliver. (Stage 02's uniwheel_env.py still defaults to 0.1, matching the reference repo,
   since that run was already in progress when this was noticed - low-impact given the small
   coefficient relative to pitch/roll's 5.0, not worth restarting for, but corrected here since
   stage 04 hasn't started.)
"""

# Standard libraries
from dataclasses import dataclass
import math
from pathlib import Path

# Third-party libraries
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import mujoco.viewer


@dataclass
class DomainRandomConfig:
    """
    Configuration for domain randomization in UniWheelEnv. Pass an instance to UniWheelEnv's
    domain_rand parameter to enable. All values default to disabled (0.0, False, or identity
    ranges).

    Attributes:
        pitch_noise_std_dev: Std dev of Gaussian noise added to the pitch observation (IMU noise)
        roll_noise_std_dev: Std dev of Gaussian noise added to the roll observation (IMU noise)
        pitch_rate_noise_std_dev: Std dev of Gaussian noise added to pitch rate (IMU noise)
        roll_rate_noise_std_dev: Std dev of Gaussian noise added to roll rate (IMU noise)
        wheel_vel_noise_std_dev: Std dev of Gaussian noise added to both wheel velocity
                                 observations (encoder noise)
        action_delay_steps: Number of steps to delay actions (0=disabled). Simulates I2C and
                            motor response latency.
        action_delay_random: If True, randomize delay 0..action_delay_steps each episode
        motor_noise_scale: Uniform noise magnitude added to motor commands (driver noise)
        push_prob: Probability per step of applying a random external force to the chassis
        push_force_max_n: Maximum magnitude of random push force in Newtons
        mass_scale_range: (min, max) scaling factor for chassis mass each episode (payload/model
                          uncertainty)
        friction_scale_range: (min, max) scaling factor for wheel-to-ground friction each episode
                              (different floor surfaces)
        motor_gain_range: (min, max) scale on both motors' torque gain (e.g. battery sag)
        ridge_prob: Probability of applying a random torque to the *wheel* axle only, simulating
                   a tire ridge hitting the ground (not applied to reaction_wheel - see module
                   docstring)
        ridge_torque_max_nm: Max random torque applied to the wheel axle (N-m)
    """
    pitch_noise_std_dev: float = 0.0
    roll_noise_std_dev: float = 0.0
    pitch_rate_noise_std_dev: float = 0.0
    roll_rate_noise_std_dev: float = 0.0
    wheel_vel_noise_std_dev: float = 0.0
    action_delay_steps: int = 0
    action_delay_random: bool = False
    motor_noise_scale: float = 0.0
    push_prob: float = 0.0
    push_force_max_n: float = 0.0
    mass_scale_range: tuple = (1.0, 1.0)
    friction_scale_range: tuple = (1.0, 1.0)
    motor_gain_range: tuple = (1.0, 1.0)
    ridge_prob: float = 0.0
    ridge_torque_max_nm: float = 0.0


class UniWheelEnv(gym.Env):
    """
    Gymnasium environment wrapping the uni wheel MuJoCo model, with optional domain
    randomization layered on top of the same observation/action/reward structure as the
    non-randomized version in 02-balance-bot-with-ppo/uniwheel_env.py.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(
        self,
        mjcf_path="../../mechanical/urdf/uniwheel/uniwheel.xml",
        max_steps=10000,
        render_mode=None,
        alpha=0.98,
        sensor_imu_accel="imu_accel",
        sensor_imu_gyro="imu_gyro",
        sensor_wheel_vel="wheel_vel",
        sensor_reaction_wheel_vel="reaction_wheel_vel",
        actuator_wheel_motor="wheel_motor",
        actuator_reaction_wheel_motor="reaction_wheel_motor",
        wheel_joint="wheel_joint",
        reaction_wheel_joint="reaction_wheel_joint",
        alive_bonus=1.0,
        pitch_penalty_coef=5.0,
        roll_penalty_coef=5.0,
        action_penalty_coef=0.01,
        position_penalty_coef=0.01,
        yaw_penalty_coef=0.0,  # see module docstring - neither actuator can cause or correct
                                # yaw, so penalizing it gives no useful training signal here
                                # (unlike the reference repo's differential-drive bot, which
                                # really can steer yaw with its two wheels)
        wheel_speed_penalty_coef=0.001,
        tip_threshold_deg=30.0,
        domain_rand=None,
    ):
        """
        Constructor: Initialize the uni wheel balance environment.

        Args mirror workspace/software/02-balance-bot-with-ppo/uniwheel_env.py, plus:
            wheel_joint (str): MJCF name of the ground wheel joint (for applying ridge torque)
            reaction_wheel_joint (str): MJCF name of the reaction wheel joint
            domain_rand (DomainRandomConfig): Configuration for domain randomization (None to
                                              disable, matching plain UniWheelEnv's behavior)
        """
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)

        self._ground_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        assert self._ground_id != -1, "Geom 'ground' not found in MJCF"

        self._chassis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        assert self._chassis_id != -1, "Body 'chassis' not found in MJCF"

        wheel_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, wheel_joint)
        assert wheel_joint_id != -1, "Wheel joint not found in MJCF"
        self._wheel_dof_idx = self.model.jnt_dofadr[wheel_joint_id]

        reaction_wheel_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, reaction_wheel_joint
        )
        assert reaction_wheel_joint_id != -1, "Reaction wheel joint not found in MJCF"
        self._reaction_wheel_dof_idx = self.model.jnt_dofadr[reaction_wheel_joint_id]

        self.render_mode = render_mode
        self.alpha = alpha

        self.sensor_imu_accel = sensor_imu_accel
        self.sensor_imu_gyro = sensor_imu_gyro
        self.sensor_wheel_vel = sensor_wheel_vel
        self.sensor_reaction_wheel_vel = sensor_reaction_wheel_vel

        self.wheel_motor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_wheel_motor
        )
        self.reaction_wheel_motor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_reaction_wheel_motor
        )

        # Save original mass, friction, and gearing values so randomization can be applied
        # relative to the true model, not accumulated across resets
        self._chassis_mass_orig = float(self.model.body_mass[self._chassis_id])
        self._ground_friction_orig = float(self.model.geom_friction[self._ground_id, 0])
        self._wheel_gear_orig = float(self.model.actuator_gear[self.wheel_motor_id, 0])
        self._reaction_wheel_gear_orig = float(
            self.model.actuator_gear[self.reaction_wheel_motor_id, 0]
        )

        # Observation: [pitch, roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel]
        obs_low = np.array([-np.pi, -np.pi, -20.0, -20.0, -50.0, -50.0], dtype=np.float32)
        obs_high = np.array([ np.pi,  np.pi,  20.0,  20.0,  50.0,  50.0], dtype=np.float32)
        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)

        actions_low = np.array([-1.0, -1.0], dtype=np.float32)
        actions_high = np.array([1.0, 1.0], dtype=np.float32)
        self.action_space = spaces.Box(actions_low, actions_high, dtype=np.float32)

        self.alive_bonus = alive_bonus
        self.pitch_penalty_coef = pitch_penalty_coef
        self.roll_penalty_coef = roll_penalty_coef
        self.action_penalty_coef = action_penalty_coef
        self.position_penalty_coef = position_penalty_coef
        self.yaw_penalty_coef = yaw_penalty_coef
        self.wheel_speed_penalty_coef = wheel_speed_penalty_coef

        self.tip_threshold_deg = tip_threshold_deg
        self.max_steps = max_steps

        # Domain randomization config (None = fully disabled, behaves like plain UniWheelEnv)
        self.dr = domain_rand

        # Action delay buffer
        self._action_delay = 0
        self._action_buffer = []

        self._pitch = 0.0
        self._roll = 0.0

        self._viewer = None
        self._step = 0

    def _get_obs(self):
        """
        Read sensor data from MuJoCo and return the observation vector, with optional Gaussian
        noise applied per self.dr.

        Returns:
            np.ndarray: [pitch, roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel]
        """
        accel_x, accel_y, accel_z = self.data.sensor(self.sensor_imu_accel).data
        pitch_rate = self.data.sensor(self.sensor_imu_gyro).data[1]
        roll_rate = self.data.sensor(self.sensor_imu_gyro).data[0]

        accel_pitch = -1 * math.atan2(accel_x, accel_z)
        accel_roll = math.atan2(accel_y, accel_z)

        self._pitch = self.alpha * (self._pitch + pitch_rate * self.model.opt.timestep) + \
                      (1 - self.alpha) * accel_pitch
        self._roll = self.alpha * (self._roll + roll_rate * self.model.opt.timestep) + \
                     (1 - self.alpha) * accel_roll

        wheel_vel = self.data.sensor(self.sensor_wheel_vel).data[0]
        reaction_wheel_vel = self.data.sensor(self.sensor_reaction_wheel_vel).data[0]

        obs = np.array(
            [self._pitch, self._roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel],
            dtype=np.float32,
        )

        if self.dr is not None:
            if self.dr.pitch_noise_std_dev > 0.0:
                obs[0] += self.np_random.normal(0.0, self.dr.pitch_noise_std_dev)
            if self.dr.roll_noise_std_dev > 0.0:
                obs[1] += self.np_random.normal(0.0, self.dr.roll_noise_std_dev)
            if self.dr.pitch_rate_noise_std_dev > 0.0:
                obs[2] += self.np_random.normal(0.0, self.dr.pitch_rate_noise_std_dev)
            if self.dr.roll_rate_noise_std_dev > 0.0:
                obs[3] += self.np_random.normal(0.0, self.dr.roll_rate_noise_std_dev)
            if self.dr.wheel_vel_noise_std_dev > 0.0:
                obs[4] += self.np_random.normal(0.0, self.dr.wheel_vel_noise_std_dev)
                obs[5] += self.np_random.normal(0.0, self.dr.wheel_vel_noise_std_dev)

        return obs

    def reset(self, seed=None, options=None):
        """
        Resets the environment and bot to an initial state, applying domain randomization (mass/
        friction/motor-gain scaling, action delay setup) per self.dr if set.

        Returns:
            observation (np.ndarray): Initial observation
            info (dict): Empty, no extra info returned on reset
        """
        super().reset(seed=seed)

        if self.dr is not None and self.dr.mass_scale_range != (1.0, 1.0):
            scale = self.np_random.uniform(*self.dr.mass_scale_range)
            self.model.body_mass[self._chassis_id] = scale * self._chassis_mass_orig

        if self.dr is not None and self.dr.friction_scale_range != (1.0, 1.0):
            scale = self.np_random.uniform(*self.dr.friction_scale_range)
            self.model.geom_friction[self._ground_id, 0] = scale * self._ground_friction_orig

        if self.dr is not None and self.dr.motor_gain_range != (1.0, 1.0):
            scale = self.np_random.uniform(*self.dr.motor_gain_range)
            self.model.actuator_gear[self.wheel_motor_id, 0] = scale * self._wheel_gear_orig
            self.model.actuator_gear[self.reaction_wheel_motor_id, 0] = \
                scale * self._reaction_wheel_gear_orig

        # Clear any applied forces from the previous episode
        self.data.xfrc_applied[self._chassis_id, :] = 0.0
        self.data.qfrc_applied[self._wheel_dof_idx] = 0.0
        self.data.qfrc_applied[self._reaction_wheel_dof_idx] = 0.0

        mujoco.mj_resetData(self.model, self.data)

        self._pitch = 0.0
        self._roll = 0.0

        # Impart an initial angular velocity around both pitch and roll axes so the agent learns
        # to recover from disturbances on either axis (and combinations of both)
        # Note: for this model's free joint, qvel[3] = wx (roll rate), qvel[4] = wy (pitch rate)
        self.data.qvel[3] += self.np_random.uniform(-0.5, 0.5)
        self.data.qvel[4] += self.np_random.uniform(-0.5, 0.5)

        mujoco.mj_forward(self.model, self.data)

        self._step = 0

        if self.dr is not None:
            if self.dr.action_delay_steps > 0 and self.dr.action_delay_random:
                self._action_delay = self.np_random.integers(0, self.dr.action_delay_steps + 1)
            else:
                self._action_delay = self.dr.action_delay_steps
            self._action_buffer = [
                np.zeros(self.action_space.shape) for _ in range(self._action_delay)
            ]

        return self._get_obs(), {}

    def step(self, action):
        """
        Advance the simulation by one step and return the result, applying action delay, motor
        noise, random pushes, and ridge torque per self.dr if set.

        Args:
            action (np.ndarray): [wheel_torque, reaction_wheel_torque], normalized to [-1, 1]

        Returns:
            obs, reward, terminated, truncated, info - same shape as plain UniWheelEnv
        """
        if self.dr is not None and self._action_delay > 0:
            self._action_buffer.append(action.copy())
            action = self._action_buffer.pop(0)

        if self.dr is not None and self.dr.motor_noise_scale > 0.0:
            noise = self.np_random.uniform(
                -self.dr.motor_noise_scale, self.dr.motor_noise_scale, size=action.shape
            )
            action = np.clip(action + noise, -1.0, 1.0)

        self.data.ctrl[self.wheel_motor_id] = action[0]
        self.data.ctrl[self.reaction_wheel_motor_id] = action[1]

        self.data.xfrc_applied[self._chassis_id, :] = 0.0
        self.data.qfrc_applied[self._wheel_dof_idx] = 0.0
        self.data.qfrc_applied[self._reaction_wheel_dof_idx] = 0.0

        if self.dr is not None and self.dr.push_prob > 0.0:
            if self.np_random.random() < self.dr.push_prob:
                push_x = self.np_random.uniform(-self.dr.push_force_max_n, self.dr.push_force_max_n)
                push_y = self.np_random.uniform(-self.dr.push_force_max_n, self.dr.push_force_max_n)
                self.data.xfrc_applied[self._chassis_id, 0] = push_x
                self.data.xfrc_applied[self._chassis_id, 1] = push_y

        # Ridge torque: wheel axle only, never reaction_wheel - see module docstring
        if self.dr is not None and self.dr.ridge_prob > 0.0:
            if self.np_random.random() < self.dr.ridge_prob:
                ridge_torque = self.np_random.uniform(
                    -self.dr.ridge_torque_max_nm, self.dr.ridge_torque_max_nm
                )
                self.data.qfrc_applied[self._wheel_dof_idx] = ridge_torque

        mujoco.mj_step(self.model, self.data)
        self._step += 1

        obs = self._get_obs()
        pitch, roll = obs[0], obs[1]
        wheel_vel, reaction_wheel_vel = obs[4], obs[5]

        pitch_penalty = self.pitch_penalty_coef * pitch**2
        roll_penalty = self.roll_penalty_coef * roll**2
        action_penalty = self.action_penalty_coef * np.sum(np.asarray(action) ** 2)
        x_pos = self.data.qpos[0]
        y_pos = self.data.qpos[1]
        position_penalty = self.position_penalty_coef * (x_pos**2 + y_pos**2)
        yaw_rate = self.data.qvel[5]
        yaw_penalty = self.yaw_penalty_coef * abs(yaw_rate)
        wheel_speed_penalty = self.wheel_speed_penalty_coef * \
            (wheel_vel**2 + reaction_wheel_vel**2)

        reward = self.alive_bonus - pitch_penalty - roll_penalty - action_penalty \
            - position_penalty - yaw_penalty - wheel_speed_penalty

        terminated = (abs(pitch) > math.radians(self.tip_threshold_deg)) or \
                     (abs(roll) > math.radians(self.tip_threshold_deg))
        truncated = self._step >= self.max_steps

        return obs, reward, terminated, truncated, {}

    def render(self):
        """
        Render the current simulation state to the MuJoCo viewer window.
        """
        if self.render_mode != "human":
            return

        if self._viewer is None:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self._viewer.cam.lookat[:] = [0, 0, 0.08]
            self._viewer.cam.distance = 0.8
            self._viewer.cam.azimuth = 45
            self._viewer.cam.elevation = -25

        self._viewer.sync()

    def close(self):
        """
        Clean up the viewer and simulation resources. Automatically called by Gymnasium when
        the environment is done.
        """
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
