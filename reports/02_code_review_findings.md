# VNAV Workspace — Code Review & Risk Findings

Companion to [01_project_overview.md](./01_project_overview.md). Severity: 🔴 high · 🟠 medium · 🟡 low.

## Summary

| # | Sev | Area | Finding |
|---|---|---|---|
| 1 | 🔴 | Repo | About a month of work (2 packages, gains, trajectory rewrite) is **not in git** |
| 2 | 🔴 | Controller | The control loop publishes rotor commands from **uninitialized state** before the first messages arrive |
| 3 | 🟠 | Controller ↔ TESSE | The **frame of the TESSE odometry twist** is ambiguous (the bridge says body frame, the controller assumes world frame) |
| 4 | 🟠 | Launch | `controller_node` is commented out of the launch file; `use_sim_time` is not passed to it |
| 5 | 🟠 | Gazebo | `lab3_keyboard` is not installed as an entry point, and it competes with the controller on `rotor_speed_cmds` |
| 6 | 🟠 | Gazebo | The simulator reuses the controller's F2W, so **mixer errors cannot show up in Gazebo** |
| 7 | 🟡 | Controller | `F2W.inverse()` runs 1000×/s; the parameter-failure branch can never run |
| 8 | 🟡 | TESSE bridge | `quadrotor_control_interface` never receives the YAML params (namespace mismatch) |
| 9 | 🟡 | Packaging | Missing or extra dependencies; placeholder `TODO` metadata |
| 10 | 🟡 | Repo | 96 MB Unity binary, Python 2 `.pyc` files and `egg-info` are tracked in git |
| 11 | 🟡 | Environment | The workspace lives on OneDrive/NTFS (`/mnt/c`), so builds are slow and sync is risky |

---

## Details

### 1 🔴 Uncommitted and untracked work
`git status` shows:
- **Modified:** [params.yaml](../src/controller_pkg/config/params.yaml), [traj_tracking.launch.py](../src/controller_pkg/launch/traj_tracking.launch.py), [traj_publisher.cpp](../src/controller_pkg/src/traj_publisher.cpp) (+96/−104)
- **Untracked:** `src/gazebo_quadrotor_pkg/`, `src/tesse_foxglove_pkg/`, `foxglove.json`

The last commit is from 2026-09-06. The tuned gains are the result of real tuning effort and should be committed.

```bash
git add src/gazebo_quadrotor_pkg src/tesse_foxglove_pkg foxglove.json src/controller_pkg
git commit -m "Tune gains, robust traj publisher, add Gazebo sim + Foxglove viz"
```

### 2 🔴 Control loop runs on uninitialized state
[controller_node.cpp L137-L139](../src/controller_pkg/src/controller_node.cpp#L137-L139) starts a 1 kHz timer right away. Eigen's `Vector3d`/`Matrix3d` and `double yawd` have **no default value**, so until both `/current_state` and `/desired_state` have arrived, `controlLoop()` computes on garbage. The result can be NaN or huge rotor speeds, which go straight to Unity or Gazebo. This can explain flaky first seconds or a drone that is "launched into space" at startup.

```cpp
bool have_state_ = false, have_desired_ = false;   // set true in each callback
void controlLoop() {
  if (!have_state_ || !have_desired_) return;
  ...
}
```

Also guard `A.normalized()`: when ‖A‖ ≈ 0 (free fall commanded), `b3d` becomes NaN.

### 3 🟠 Velocity frame of the TESSE odometry
- In the controller, linear `v` is treated as **world frame** (`ev = v − vd`, with `vd` in world), and angular velocity is rotated with `omega = Rᵀ·ω_world` ([L208-L219](../src/controller_pkg/src/controller_node.cpp#L208-L219)). This matches the lab handout's caveat.
- [`process_metadata`](../src/tesse_ros_bridge/tesse_ros_bridge/utils.py#L586-L619) and [`metadata_to_odom`](../src/tesse_ros_bridge/tesse_ros_bridge/utils.py#L272-L279) say *"Velocities … are all in the body frame"*. Angular velocity is computed with a log map of `prevRᵀ·R`, which is a body-frame quantity.
- In **Gazebo**, `gazebo_ros_p3d` reports world-frame twist, so the controller is correct there.

If TESSE really reports body frame, the fix would be `v = R * v_body` and `omega = omega_body`. With yaw sweeping 360° on the circle, a linear-velocity frame error would visibly degrade tracking, while a hover test would hide it. **Check it with a test:** fly the circle in TESSE and compare `/current_state` twist against finite differences of the pose. Treat this as unverified until that test is done. Do not change the code yet.

### 4 🟠 Launch configuration
[traj_tracking.launch.py L20-L26](../src/controller_pkg/launch/traj_tracking.launch.py#L20-L26) only starts `traj_publisher`. When you re-enable the controller, pass both the YAML and `use_sim_time`:

```python
parameters=[join(get_package_share_directory('controller_pkg'), 'config/params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
```

The controller uses a **wall** timer (1 kHz). With TESSE running slower than real time, the loop then runs many times per sim step. This is harmless (the output stays the same), but it wastes CPU.

### 5 🟠 `lab3_keyboard` integration
- [setup.py](../src/gazebo_quadrotor_pkg/setup.py) registers only `quadrotor_bridge` and `lab3_viz_publisher`, so `ros2 run gazebo_quadrotor_pkg lab3_keyboard` fails. Add `'lab3_keyboard = gazebo_quadrotor_pkg.lab3_keyboard:main'`.
- It imports `gazebo_msgs`, but [package.xml](../src/gazebo_quadrotor_pkg/package.xml) does not declare it.
- Once armed, it publishes hover speeds at 50 Hz on the **same topic** as `controller_node` (1 kHz). The bridge applies whichever message arrived last, which causes jitter. Add a "disarm" key or don't run both together.
- Its default respawn height is `initial_z = 0.5`, while the launch file spawns at `spawn_z = 2.0`.

### 6 🟠 Gazebo cannot validate the mixer
[quadrotor_bridge.py L14-L25](../src/gazebo_quadrotor_pkg/gazebo_quadrotor_pkg/quadrotor_bridge.py#L14-L25) copies the controller's `F2W`. Because `F2W·F2W⁻¹ = I`, any sign or rotor-order error in the matrix cancels out in Gazebo but fails in TESSE. Gazebo is good for testing the **control law**, not the **allocation**. Consider putting the true TESSE rotor geometry in the bridge and stating that in the package README.

### 7 🟡 Minor controller issues
- `F2W.inverse()` is constant. Compute it once in the constructor ([L323](../src/controller_pkg/src/controller_node.cpp#L323)).
- In Humble, `declare_parameter<double>("kx")` with no default **throws** `NoParameterOverrideProvided` when the YAML is missing, so the friendly `exit(1)` branch ([L145-L150](../src/controller_pkg/src/controller_node.cpp#L145-L150)) never runs.
- Absolute topic names (`/desired_state`, `/rotor_speed_cmds`) prevent namespacing for multiple drones. Relative names work the same today.
- `#include <std_msgs/msg/string.hpp>` in `traj_publisher.cpp` is unused, and that target lists Eigen/mav_msgs/nav_msgs dependencies it doesn't need.

### 8 🟡 TESSE control interface params
The [launch YAML](../src/tesse_ros_bridge/launch/tesse_quadrotor_bridge.launch.yaml) names the node `quadrotor_control_interface`, but the param files are keyed under `tesse_ros_bridge:`. The node therefore runs on hard-coded defaults; for example `use_broadcast=False` instead of `true`. This works today because only localhost is used.

### 9 🟡 Packaging metadata
- `controller_pkg/package.xml`: `rosidl_default_generators` is listed as a buildtool dependency but nothing uses it; the description says "TODO".
- `tesse_ros_bridge/package.xml` declares **no runtime dependencies** (`mav_msgs`, `tesse_msgs`, `cv_bridge`, `ackermann_msgs`, `rospy2`, `scipy`), so a fresh `rosdep install` will not pull them in.

### 10 🟡 Repository bloat
- `vnav_builds/` (96 MB, 203 files including `.debug` symbols) is committed. Use Git LFS or a download script.
- `src/tesse-interface/src/tesse/*.pyc` (Python 2), `tesse.egg-info/` and `build/lib/` are tracked even though `.gitignore` excludes `*.py[cod]`. Remove them with `git rm --cached`.
- `__pycache__` folders inside `src/*/launch` are correctly ignored.

### 11 🟡 Environment
The workspace is under `/mnt/c/.../OneDrive/...`. Every build goes through the WSL↔NTFS 9P layer: `mav_comm` took **315 s**, and a single-file `controller_pkg` takes 30–50 s. OneDrive also syncs `build/` and `install/` (~50 MB of churn), and it can lock files mid-build. Moving to `~/vnav_ws` (ext4) typically makes builds **5–10× faster**. Git, not OneDrive, should be the backup.

---

## Recommended action order

1. **Commit everything** (#1).
2. Add **initialization guards** to `controlLoop()` (#2). This is a 5-line fix that removes startup instability.
3. **Re-enable the controller in the launch file** with `use_sim_time` (#4).
4. Run the **TESSE twist-frame test** (#3) and record the result.
5. Fix the `lab3_keyboard` entry point and dependency (#5).
6. Housekeeping: move off OneDrive, use LFS for `vnav_builds`, remove the stale `.pyc` files (#10, #11).

## What's done well
- The controller math is correct for the z-up convention: the gravity sign, `F = A·Re₃`, the e_R vee map, the gyroscopic term, and signed square roots for negative thrust.
- Gains are well damped (ζ ≈ 1) after tuning.
- The trajectory publisher rewrite is robust: sim-clock aware, phase accumulator, clock-jump hold, analytic feed-forward.
- The Gazebo substitute is a clean minimal design (wrench-level actuation plus ground-truth odometry). It is useful for iterating without the Unity binary, and the Lab 4 world/gate markers are already in place.
- The Foxglove tooling is a documented, configurable visualization that works with both backends.
