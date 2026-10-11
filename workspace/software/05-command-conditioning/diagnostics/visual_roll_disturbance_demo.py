"""
Clean visual demo: cmd_vel locked at 0 (no driving), a clearly visible sustained roll push
every ~4s, real MuJoCo viewer, real-time paced - to see the disturbance-and-recovery in
isolation, with no cmd_vel-driven wheel motion to confuse it with.

Why this exists: during normal training-run rendering, the ground wheel is constantly busy
driving forward/backward on command (part of this stage's velocity-tracking training) - a big,
continuous motion that's easy to mistake for "disturbance in the wheel direction" even though
it's unrelated to disturbance at all. This script removes that entirely (cmd_vel is hardcoded to
0 every step) so the only motion you'll see is the push-and-recovery itself.

What to expect: every ~4 seconds, the robot gets shoved sideways (alternating left/right) with
enough force to visibly lean, and you should see the reaction wheel (the small enclosed disc,
not the ground-contact wheel) do the work of bringing it back upright - the ground wheel should
stay essentially still throughout, since this uses phase-10's checkpoint and cmd_vel=0 the whole
time (no reason for it to drive anywhere).

Run with: python3 diagnostics/visual_roll_disturbance_demo.py   (from 05-command-conditioning/)
"""
import math, sys, time
import mujoco
import mujoco.viewer
import numpy as np
import torch
sys.path.append("..")
from rl.ppo_trainer import PPOConfig, Agent

CKPT = "runs/UniWheel-v0__uniwheel-ppo-cmd-phase-10__42__1788754064/best_model.cleanrl_model"
model = mujoco.MjModel.from_xml_path("../../mechanical/urdf/uniwheel/uniwheel.xml")
data = mujoco.MjData(model)
chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
wheel_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_motor")
rw_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "reaction_wheel_motor")

class _Spaces: pass
dummy = _Spaces()
dummy.single_observation_space = type("S", (), {"shape": (7,)})()
dummy.single_action_space = type("S", (), {"shape": (2,)})()
agent = Agent(dummy, PPOConfig(actor_hidden_layers=2, actor_hidden_size=48, critic_hidden_layers=2, critic_hidden_size=48))
agent.load_state_dict(torch.load(CKPT, map_location="cpu", weights_only=True))
agent.eval()

mujoco.mj_resetData(model, data)
pitch = roll = 0.0
PUSH_EVERY = 800   # ~4s
PUSH_DURATION = 40  # 0.2s
PUSH_FORCE = 0.4

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [0, 0, 0.08]
    viewer.cam.distance = 0.8
    viewer.cam.azimuth = 0
    viewer.cam.elevation = -10

    t = 0
    while viewer.is_running() and t < 4000:
        step_start = time.time()
        data.xfrc_applied[chassis_id, :] = 0.0
        phase = t % PUSH_EVERY
        if phase < PUSH_DURATION:
            data.xfrc_applied[chassis_id, 1] = PUSH_FORCE if (t // PUSH_EVERY) % 2 == 0 else -PUSH_FORCE

        ax, ay, az = data.sensor("imu_accel").data
        pr = data.sensor("imu_gyro").data[1]
        rr = data.sensor("imu_gyro").data[0]
        ap = -1*math.atan2(ax, az); ar = math.atan2(ay, az)
        pitch = 0.98*(pitch+pr*model.opt.timestep) + 0.02*ap
        roll = 0.98*(roll+rr*model.opt.timestep) + 0.02*ar
        wv = data.sensor("wheel_vel").data[0]
        rwv = data.sensor("reaction_wheel_vel").data[0]
        obs = np.array([pitch, roll, pr, rr, wv, rwv, 0.0], dtype=np.float32)  # cmd_vel locked at 0
        with torch.no_grad():
            act = agent.actor_mean(torch.as_tensor(obs).unsqueeze(0)).squeeze(0).numpy()
        act = np.clip(act, -1, 1)
        data.ctrl[wheel_motor_id] = act[0]
        data.ctrl[rw_motor_id] = act[1]
        mujoco.mj_step(model, data)
        viewer.sync()
        t += 1
        slack = model.opt.timestep - (time.time() - step_start)
        if slack > 0:
            time.sleep(slack)

print("demo finished")
