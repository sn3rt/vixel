# Installation

## Supported platform

Vixel targets ROS 2 Lyrical on Ubuntu 26.04. Other Linux distributions may
work, but are not part of the current test matrix.

Install ROS 2 Lyrical and the standard development tools using the official ROS
instructions. Then clone the workspace and resolve package dependencies:

```bash
git clone https://github.com/sn3rt/vixel.git
cd vixel
rosdep install --from-paths . --ignore-src -r -y --rosdistro lyrical \
  --skip-keys "ament_pytest gjs"
colcon build --symlink-install
source install/setup.bash
```

Aravis 0.8.34 or newer can be supplied by the system. If it is absent, the
`vixel_aravis_vendor` package downloads and builds the pinned release. Building
that fallback requires Git, Meson, GLib, GObject, GIO, and libxml2 development
packages.

## Project environment with direnv

Direnv is optional but convenient for development:

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
exec bash
cd vixel
direnv allow
```

The checked-in `.envrc` loads `/opt/ros/lyrical`, the built overlay when it
exists, and an optional untracked `.env`. See `.env.example` for overrides.

## Host configuration

Copy the installed example and replace every interface, MAC address, and subnet:

```bash
sudo install -d -m 0755 /etc/vixel /var/lib/vixel
sudo install -m 0644 \
  install/vixel/share/vixel/config/machine.example.yaml \
  /etc/vixel/machine.yaml
sudoedit /etc/vixel/machine.yaml
ros2 run vixel_manager vixel -- config paths
ros2 run vixel_manager vixel -- config validate
```

The runtime inventory is created at `/var/lib/vixel/inventory.yaml`. In an
unprivileged development environment, Vixel falls back to
`$XDG_STATE_HOME/vixel/inventory.yaml`.

## Build and test

```bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

The hardware-independent test suite does not require connected cameras.
