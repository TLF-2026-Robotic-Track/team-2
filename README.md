# Duckiebot Examples

Four working examples for the Duckiebot, in one ROS2 workspace:

| Package | What it does |
|---|---|
| `blinker` | Blinks the LEDs red, green, blue. |
| `camera` | Saves pictures from the camera into `images/`. |
| `control_room` | Reads the arrow keys in the terminal. No GUI. |
| `robot` | Movements. Takes a command, drives the wheels and the LEDs. |
| `master_launch` | Starts several nodes with one command. |

## Step 1. Get on the robot

```bash
ssh duckie@duckie0X.local
```

Then get this code onto the robot:

```bash
git clone <SSH LINK FROM GITHUB>
cd <FOLDER>
```

Later, to get new changes: `git pull`.

## Step 2. Set the two names

```bash
export VEHICLE_NAME=duckie03      # the name written on your robot
export USER_NAME=vasya            # your own name, letters and digits only
```

Do this in **every** new terminal. If you forget, the nodes stop with a message
telling you which one is missing. 

## Step 3. Build

```bash
make build
```

This builds the Docker image, then builds the workspace inside it.

You do **not** have to run it again after editing a `.py` or a launch file. The
build links those files instead of copying them, so it is enough to stop the
node with `Ctrl+C` and start it again.

Run `make build` again when you:

- add or rename a file, or add a package
- change `setup.py`, `setup.cfg` or `package.xml`
- change `requirements-apt.txt` or `requirements-python.txt`
- `git pull` and any of the above came with it (when in doubt, just run it, it
  takes a second)


## Step 4. Run

```bash
make run
```

You are now inside the container, in `/workspace`. ROS2 and the workspace are
already sourced for you. Check that you can see the robot:

```bash
ros2 topic list
```

You should see a list of topics starting with `/duckie0X/`. If the list is
empty, see "When it does not work".

## How to run one node

```bash
ros2 run blinker blinker.py         # LEDs blink
ros2 run camera camera.py           # pictures appear in images/
```

`Ctrl+C` stops the node.

Pattern is always `ros2 run <package> <node file>`.

## How to run several nodes

Two ways.

**One command, one terminal:**

```bash
ros2 launch master_launch keyboard.launch.xml    # control_room + robot
ros2 launch master_launch demo.launch.xml        # control_room + robot + camera
```

Now drive:

| Key | Action |
|---|---|
| arrow up, `w` | forward |
| arrow down, `s` | backward |
| arrow left, `a` | turn left |
| arrow right, `d` | turn right |
| space | stop |
| `1` `2` `3` | LEDs red, green, blue |
| `0` | LEDs white |
| `q` | quit, and stop the robot |

The wheels keep going after a key press. Press space to stop.

**Or one node per terminal.** Better when you want to read the logs of each node
separately. `tmux` is already inside the container, so start one container and
split it:

```bash
export VEHICLE_NAME=duckie03
export USER_NAME=vasya
make run
tmux   
```

Export the two names **BEFORE** you type `tmux`, then every pane inherits them.
Each pane is a new shell, so ROS2 and the workspace get sourced automatically.

Now run `ros2 run robot main.py` in one pane, `ros2 run control_room main.py` in
another.

If you prefer separate SSH sessions to tmux panes, use `make run` in the first
one and `make shell` in the rest. `make shell` opens another shell in the
container that is **already running**, instead of starting a second container.
You have to export the two names again in each `make shell`.

## Where to change what

| You want to change | Open this file | Look for |
|---|---|---|
| LED colors, blink speed | `src/blinker/blinker/blinker.py` | `COLORS`, `PERIOD` |
| How often pictures are saved | `src/camera/camera/camera.py` | `EVERY_NTH` |
| Where pictures go | `src/camera/camera/camera.py` | `OUTPUT_DIR` |
| Which key does what | `src/control_room/control_room/main.py` | `KEY_MAP` |
| Driving speed, turning speed | `src/robot/robot/main.py` | `SPEED`, `TURN_SPEED` |
| What a command does | `src/robot/robot/main.py` | `WHEELS`, `LIGHTS` |
| Which nodes start together | `src/master_launch/launch/*.launch.xml` | `<node>`, `<include>` |
| Extra apt packages | `requirements-apt.txt` | names only, no comments |
| Extra Python packages | `requirements-python.txt` | one per line |

Both requirements files are empty, because these examples need nothing extra.

Changed a `.py`? Restart the node, that is all. Changed anything else in the
table? `make build` first.

## How to get the pictures onto your computer

The camera node writes into the `images/` folder of this repo on the robot. Run
this **ON YOUR OWN COMPUTER**, not on the robot:

```bash
./get_images.sh duckie03
```

Or in one line, without the script:

```bash
scp 'duckie@duckie03.local:~/DuckieExamples/images/*.jpg' .
```

Mind the quotes: they stop your own shell from expanding `*.jpg` before it
reaches the robot.

## How to add your own node

1. Copy a package folder, for example `src/blinker`, to `src/my_thing`.
2. Rename the inner folder, the `.py` file, and the file in `resource/`.
3. Replace the package name in `package.xml`, `setup.py` and `setup.cfg`.
4. Add your node file to `data_files` in `setup.py`:
   `('lib/' + package_name, ['my_thing/my_thing.py'])`.
5. `chmod +x src/my_thing/my_thing/my_thing.py`
6. `make build`, then `ros2 run my_thing my_thing.py`.

## When it does not work

| Message or symptom | Fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `Package 'blinker' not found` | You did not build, or you did not source. Run `make build`, then inside the container `source install/local_setup.bash`. |
| `can't open file '/opt/ros/humble/_local_setup_util_sh.py'` | You sourced `install/setup.bash`. Use `install/local_setup.bash` instead. The image holds two ROS prefixes that name that helper file differently, and `setup.bash` re-sources both. |
| Edited a `.py` but nothing changed | You did not restart the node. If you renamed or added a file, run `make build`. |
| `ModuleNotFoundError: duckietown_msgs` | Same as above. |
| `VEHICLE_NAME is not set` | Do Step 2 again in this terminal. |
| `ros2 topic list` shows nothing from the robot | The container must run with `--network=host`. Use `make run`, not your own `docker run`. |
| Nodes start, robot does not move | Watch the commands arrive: `ros2 topic echo /$VEHICLE_NAME/wheels_cmd`. If they arrive and nothing turns, the robot is in emergency stop or the battery is low. |
| Arrow keys do nothing | You must type into the same terminal that started `control_room`. Also check `ros2 topic echo /$USER_NAME/$VEHICLE_NAME/command`. |
| LEDs flicker or ignore you | Two nodes are publishing to `led_pattern`. Stop `blinker` while driving. |
| Build breaks after renaming things | `make clean`, then `make build`. |
| One package fails and blocks the rest | Build only what you need: `colcon build --packages-select blinker`. |

## Cheat sheet

```bash
# Topics
ros2 topic list                                  # what exists
ros2 topic echo /$VEHICLE_NAME/wheels_cmd        # watch messages
ros2 node list                                   # what is running

# Drive without any of our nodes
ros2 topic pub --once /$VEHICLE_NAME/wheels_cmd \
  duckietown_msgs/msg/WheelsCmdStamped "{vel_left: 0.4, vel_right: 0.4}"

# All LEDs red, without any of our nodes
ros2 topic pub --once /$VEHICLE_NAME/led_pattern \
  duckietown_msgs/msg/LEDPattern \
  "{rgb_vals: [{r: 1.0, a: 1.0}, {r: 1.0, a: 1.0}, {r: 1.0, a: 1.0}, {r: 1.0, a: 1.0}, {r: 1.0, a: 1.0}]}"

# See what the camera sees, from a computer with a screen
ros2 run rqt_image_view rqt_image_view

# Building
colcon build                                     # everything
colcon build --packages-select blinker           # one package
source install/local_setup.bash                  # after every build
make clean                                       # delete build/ install/ log/
```

Wheel speeds go from `-1.0` to `1.0`. Positive is forward, `0.0` stops.

Useful robot topics: `range` (distance sensor), `image/compressed` (camera),
`tick` (wheel encoders), `temperature`, `wheels_cmd`, `led_pattern`,
`emergency_stop`.
