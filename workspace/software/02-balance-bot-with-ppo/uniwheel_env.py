"""
Gymnasium wrapper for MuJoCo simulator that loads and runs the environment for the uni wheel
reaction-wheel balancing robot (one ground wheel for pitch, one reaction/flywheel for roll).

Adapted from the reference repo's BalanceBotEnv (2-wheel differential-drive bot, pitch only).
Two things are structurally different here, both driven by findings in stage 01
(see workspace/software/01-test-model-in-mujoco/STAGE01_FINDINGS.md):

  1. Two independently-actuated axes (pitch via the ground wheel, roll via the reaction wheel)
     instead of one, so both angles/rates and both actuators are tracked, not just pitch.
  2. The reaction wheel's own spin speed is included in the observation. A hand-tuned PID
     that ignores it works fine short-term but has no way to avoid the reaction wheel's finite
     RPM ceiling (see docs/feasibility_analysis.md's momentum-saturation section) or the
     gyroscopic coupling between the two wheels that broke stage 01's combined-disturbance
     recovery. A learned policy that can *see* wheel speed at least has the information needed
     to manage it, even though we're not otherwise engineering saturation-avoidance behavior in.

NOTE on yaw_penalty_coef (left at 0.1, matching the reference repo, below): that default is
actually wrong for this robot and was fixed to 0.0 in stage 04's uniwheel_env_dr.py - the
reference repo's differential-drive bot can really steer yaw with its two wheels, so penalizing
yaw drift discourages a real, achievable behavior there. Neither of our actuators has any torque
component about the yaw axis, so any yaw here is a passive side effect, not something the policy
can cause or correct - penalizing it gives no useful signal. Left unchanged here (rather than
edited to match) because a training run using this exact file was already in progress when this
was noticed, and the coefficient is small relative to pitch/roll's 5.0 - not worth restarting for.
"""

# Standard libraries
import math
from pathlib import Path

# Third-party libraries
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import mujoco.viewer


class UniWheelEnv(gym.Env):
    """
    Gymnasium environment wrapping the uni wheel MuJoCo model so it can be driven with the
    standard gymnasium API (step(), reset(), action_space, etc.).
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
        alive_bonus=1.0,
        pitch_penalty_coef=5.0,
        roll_penalty_coef=5.0,
        action_penalty_coef=0.01,
        position_penalty_coef=0.01,
        yaw_penalty_coef=0.1,
        wheel_speed_penalty_coef=0.001,
        tip_threshold_deg=30.0,
    ):
        """
        Constructor: Initialize the uni wheel balance environment.

        Args:
            mjcf_path (str or Path): Path to the MuJoCo MJCF model file (XML)
            max_steps (int): Max number of simulation steps per episode before resetting
            render_mode (str or None): Visual output, one of [None, "human", "rgb_array"]
            alpha (float): Complementary filter coefficient, higher = more gyro, lower = more accel
            sensor_imu_accel (str): MJCF name of the IMU accelerometer sensor
            sensor_imu_gyro (str): MJCF name of the IMU gyroscope sensor
            sensor_wheel_vel (str): MJCF name of the ground wheel velocity sensor
            sensor_reaction_wheel_vel (str): MJCF name of the reaction wheel velocity sensor
            actuator_wheel_motor (str): MJCF name of the ground wheel motor actuator
            actuator_reaction_wheel_motor (str): MJCF name of the reaction wheel motor actuator
            alive_bonus (float): Reward given each step the robot stays upright
            pitch_penalty_coef (float): Scales the pitch^2 penalty, encourages staying upright
            roll_penalty_coef (float): Scales the roll^2 penalty, encourages staying upright
            action_penalty_coef (float): Scales the action^2 penalty, discourages jittery motion
            position_penalty_coef (float): Scales the position penalty (x^2 + y^2), discourages
                                           drifting from the starting position
            yaw_penalty_coef (float): Scales the abs(yaw_rate) penalty, discourages spinning
                                      around the Z axis
            wheel_speed_penalty_coef (float): Scales (wheel_vel^2 + reaction_wheel_vel^2),
                                              discourages needlessly spinning either wheel up -
                                              this is the one term the reference repo didn't
                                              need (see module docstring)
            tip_threshold_deg (float): Angle (degrees), on either axis, at which the robot is
                                       considered tipped over
        """
        # Load model into MuJoCo and get simulation state
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)

        self.render_mode = render_mode
        self.alpha = alpha

        self.sensor_imu_accel = sensor_imu_accel
        self.sensor_imu_gyro = sensor_imu_gyro
        self.sensor_wheel_vel = sensor_wheel_vel
        self.sensor_reaction_wheel_vel = sensor_reaction_wheel_vel

        # Get ID of actuators from MJCF names
        self.wheel_motor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_wheel_motor
        )
        self.reaction_wheel_motor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_reaction_wheel_motor
        )

        # Observation: [pitch, roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel]
        obs_low = np.array(
            [-np.pi, -np.pi, -20.0, -20.0, -50.0, -50.0], dtype=np.float32
        )
        obs_high = np.array(
            [ np.pi,  np.pi,  20.0,  20.0,  50.0,  50.0], dtype=np.float32
        )
        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)

        # Action: [wheel_torque, reaction_wheel_torque], normalized to [-1, 1]
        actions_low = np.array([-1.0, -1.0], dtype=np.float32)
        actions_high = np.array([1.0, 1.0], dtype=np.float32)
        self.action_space = spaces.Box(actions_low, actions_high, dtype=np.float32)

        # Save reward coefficients
        self.alive_bonus = alive_bonus
        self.pitch_penalty_coef = pitch_penalty_coef
        self.roll_penalty_coef = roll_penalty_coef
        self.action_penalty_coef = action_penalty_coef
        self.position_penalty_coef = position_penalty_coef
        self.yaw_penalty_coef = yaw_penalty_coef
        self.wheel_speed_penalty_coef = wheel_speed_penalty_coef

        self.tip_threshold_deg = tip_threshold_deg
        self.max_steps = max_steps

        # Complementary-filter state
        self._pitch = 0.0
        self._roll = 0.0

        self._viewer = None
        self._step = 0

    def _get_obs(self):
        """
        Read sensor data from MuJoCo and return the observation vector.

        Returns:
            np.ndarray: [pitch, roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel]
        """
        accel_x, accel_y, accel_z = self.data.sensor(self.sensor_imu_accel).data
        pitch_rate = self.data.sensor(self.sensor_imu_gyro).data[1]
        roll_rate = self.data.sensor(self.sensor_imu_gyro).data[0]

        # Accelerometer-derived tilt estimates (same convention as stage 01's PID notebook:
        # pitch about world Y via the ground wheel, roll about world X via the reaction wheel)
        accel_pitch = -1 * math.atan2(accel_x, accel_z)
        accel_roll = math.atan2(accel_y, accel_z)

        # Complementary filter: gyro integration for fast response, accelerometer to correct
        # long-term drift
        self._pitch = self.alpha * (self._pitch + pitch_rate * self.model.opt.timestep) + \
                      (1 - self.alpha) * accel_pitch
        self._roll = self.alpha * (self._roll + roll_rate * self.model.opt.timestep) + \
                     (1 - self.alpha) * accel_roll

        wheel_vel = self.data.sensor(self.sensor_wheel_vel).data[0]
        reaction_wheel_vel = self.data.sensor(self.sensor_reaction_wheel_vel).data[0]

        return np.array(
            [self._pitch, self._roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel],
            dtype=np.float32,
        )

    def reset(self, seed=None, options=None):
        """
        Resets the environment and bot to an initial state.

        Args:
            seed (int or None): seed for the random number generator(s)
            options (dict or None): Unused, required by the Gymnasium API

        Returns:
            observation (np.ndarray): Initial observation
            info (dict): Empty, no extra info returned on reset
        """
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        self._pitch = 0.0
        self._roll = 0.0

        # Impart an initial angular velocity around both the pitch and roll axes so the agent
        # learns to recover from disturbances on either axis (and combinations of both - the
        # thing a decoupled PID couldn't handle in stage 01).
        # Note: for this model's free joint, qvel[3] = wx (roll rate), qvel[4] = wy (pitch rate)
        self.data.qvel[3] += self.np_random.uniform(-0.5, 0.5)
        self.data.qvel[4] += self.np_random.uniform(-0.5, 0.5)

        mujoco.mj_forward(self.model, self.data)

        self._step = 0

        return self._get_obs(), {}

    def step(self, action):
        """
        Advance the simulation by one step and return the result.

        Args:
            action (np.ndarray): [wheel_torque, reaction_wheel_torque], normalized to [-1, 1]

        Returns:
            obs (np.ndarray): Observation from _get_obs()
            reward (float): Reward signal for this step
            terminated (bool): True if the robot has tipped/fallen over (either axis)
            truncated (bool): True if the episode has reached max_steps
            info (dict): Empty, no extra info returned
        """
        self.data.ctrl[self.wheel_motor_id] = action[0]
        self.data.ctrl[self.reaction_wheel_motor_id] = action[1]

        mujoco.mj_step(self.model, self.data)
        self._step += 1

        obs = self._get_obs()
        pitch, roll = obs[0], obs[1]
        wheel_vel, reaction_wheel_vel = obs[4], obs[5]

        # Reward function:
        #   alive - (A*pitch^2) - (A*roll^2) - (B*action^2) - (C*(x^2+y^2)) - D*abs(yaw_rate)
        #         - (E*(wheel_vel^2 + reaction_wheel_vel^2))
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

        # Termination (if robot tips on either axis, or we run out of time in the episode)
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
