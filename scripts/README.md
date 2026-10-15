# Running VNAV Lab 3

The run and build scripts source ROS 2 Humble and this workspace automatically. Run the examples below from the workspace root; from another directory, use the absolute path to a script. Each full stack uses a tmux session with separate windows for the simulator, trajectory, controller, and optional tools.

## Before starting

~~~bash
scripts/build.sh                       # first build, or after source code changes
~~~

You need ROS 2 Humble and tmux. Gazebo also needs Gazebo Classic and `gazebo_ros`. Unity needs a working graphical display; WSLg normally provides `DISPLAY=:0`. Start one backend at a time: both use `/current_state`, `/desired_state`, `/rotor_speed_cmds`, and `/clock`. The optional Foxglove bridges both use port 8765.

## Gazebo path

~~~bash
scripts/run_gazebo.sh                  # Gazebo, trajectory, controller
scripts/run_gazebo.sh --headless       # same ROS stack without Gazebo GUI
scripts/run_gazebo.sh --no-controller --detach  # simulator + trajectory for tuning
~~~

The Gazebo launch also starts the quadrotor bridge and visualization publisher. It spawns the quadrotor at 2 m by default. `--spawn-z 1.0` changes that height; `--lab4` selects the Lab 4 world; `--paused` starts physics paused. Optional `--foxglove` and `--rviz` start those viewers in their own tmux windows.

For manual Gazebo keyboard control, use `scripts/run_gazebo.sh --no-controller --keyboard`. In the keyboard window, W arms hover-speed commands, R respawns, and Q quits the helper. Do not arm the keyboard while the controller is running: both publish `/rotor_speed_cmds`. With the controller running, the keyboard's R key can still be used to respawn.

## Unity (TESSE) path

~~~bash
scripts/run_tesse.sh                   # Unity, TESSE bridge, trajectory, controller
scripts/run_tesse.sh --sim-wait 15     # wait longer if Unity starts slowly
scripts/run_tesse.sh --no-controller --detach  # simulator + bridge + trajectory for tuning
~~~

The bridge publishes Unity odometry on `/current_state` and forwards `/rotor_speed_cmds` to Unity. `--no-sim` connects to a Unity instance you started yourself; `scripts/stop.sh tesse` will not stop that external instance. Optional `--foxglove` starts TESSE markers and the Foxglove bridge; `--rviz` starts RViz.

**Flight status:** Unity and the ROS data path were verified to run. In a test with the current saved gains, the quadrotor flew but diverged from the circle. `scripts/run_tesse.sh` starts nodes for testing and tuning; stable tracking has not been verified. Use `--no-controller` when you only need the simulator, bridge, and trajectory.

## Tune either backend without rebuilding

Start one backend without its controller. Choose **one** of these commands:

~~~bash
scripts/run_gazebo.sh --no-controller --detach
~~~

For Unity, use `scripts/run_tesse.sh --no-controller --detach` instead. In a second terminal, run:

~~~bash
scripts/run_controller.sh                        # reads src/controller_pkg/config/params.yaml
# Ctrl+C stops this standalone controller; edit gains and run it again.
scripts/run_controller.sh kr=1.0 komega=1.0     # override syntax example, not a validated tune
~~~

`run_controller.sh` reads the **source** `params.yaml` each time it starts. Gain edits and command-line overrides require a controller restart, but no rebuild. Use only one controller at a time. Rebuild `controller_pkg` after changing C++ code; rebuild `tesse_ros_bridge` after changing its Python bridge code. `scripts/build.sh PACKAGE_NAME` builds a selected package.

You can also run only the trajectory with `scripts/run_traj.sh`, or source the environment in a terminal with `source scripts/env.sh` and use normal `ros2` commands.

## Run components in separate terminals

If you prefer logs in separate terminals instead of tmux, source the environment in each terminal that uses `ros2`. Start one simulator path:

~~~bash
source scripts/env.sh
ros2 launch gazebo_quadrotor_pkg gazebo_quadrotor.launch.py
~~~

For Unity, start the player in its own terminal with `(cd vnav_builds/lab3 && ./lab3.x86_64)`, then run `ros2 launch tesse_ros_bridge tesse_quadrotor_bridge.launch.yaml` in another sourced terminal. Run `scripts/run_traj.sh` and `scripts/run_controller.sh` in additional terminals when needed. Stop manually started processes with Ctrl+C in their terminals; `scripts/stop.sh` only manages the tmux sessions created by the full-stack scripts.

## Check the running nodes

In another terminal:

~~~bash
source scripts/env.sh
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /current_state --once
ros2 topic echo /desired_state --once
ros2 topic echo /rotor_speed_cmds --once  # only when a controller or keyboard is publishing
~~~

If Unity publishes /clock but no /current_state, inspect the bridge tmux window for a metadata error. If the bridge starts before Unity is ready, restart with a larger --sim-wait value. A paused Gazebo world will not advance /clock until unpaused.

## tmux and stopping

Without --detach, a full-stack script attaches to its tmux session. With --detach, it returns while the nodes keep running. Use Ctrl+b then a window number, or click a window with the mouse, to inspect logs. Ctrl+b d detaches; Ctrl+b [ enters scroll mode (q exits it).

~~~bash
tmux attach -t vnav_gazebo           # or vnav_tesse
scripts/stop.sh gazebo               # stop the Gazebo session
scripts/stop.sh tesse                # stop the Unity session
scripts/stop.sh                      # stop both sessions
~~~

Starting the same backend again restarts its tmux session. stop.sh sends Ctrl+C to session windows and then closes the session. It leaves simulators started outside those sessions alone. A crashed node's window stays open to show its exit code.

## Script reference

| Script | Purpose |
|---|---|
| [build.sh](build.sh) | Build all packages or selected packages; --clean removes build output first |
| [run_gazebo.sh](run_gazebo.sh) | Gazebo stack; run with --help for all options |
| [run_tesse.sh](run_tesse.sh) | Unity stack; run with --help for all options |
| [run_controller.sh](run_controller.sh) | Standalone controller with optional name=value gain overrides |
| [run_traj.sh](run_traj.sh) | Standalone trajectory publisher |
| [run_keyboard.sh](run_keyboard.sh) | Standalone Gazebo keyboard helper |
| [stop.sh](stop.sh) | Stop gazebo, tesse, or both |
| [env.sh](env.sh) | Source ROS and workspace setup into the current terminal |

For a TESSE camera image in Foxglove, set publish_third_pov: true in src/tesse_ros_bridge/config/params.yaml and rebuild tesse_ros_bridge. Foxglove connects to ws://localhost:8765; layouts are in src/gazebo_quadrotor_pkg/config/lab3_foxglove.json and src/tesse_foxglove_pkg/config/tesse_foxglove.json.
