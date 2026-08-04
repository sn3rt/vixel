# Installation

## Supported platform

Vixel targets ROS 2 Lyrical on Ubuntu 26.04. Other Linux distributions may
work, but are not part of the current test matrix.

Install ROS 2 Lyrical and the standard development tools using the official ROS
instructions. Then clone the workspace and resolve package dependencies:

```bash
sudo apt update
sudo apt install -y linuxptp
git clone https://github.com/sn3rt/vixel.git
cd vixel
rosdep install --from-paths . --ignore-src -r -y --rosdistro lyrical \
  --skip-keys "ament_pytest gjs"
colcon build --symlink-install
source install/setup.bash
```

`linuxptp` is installed explicitly because ROS rosdep currently has no key for
the Ubuntu package. Vixel's network validation and service installer check for
both `ptp4l` and `phc2sys` and print the install command when either is missing.

The setup script must match the current shell. The examples use Bash. In zsh,
use the corresponding scripts instead:

```zsh
source /opt/ros/lyrical/setup.zsh
source /home/robotti/vixel/install/setup.zsh
```

Do not source a `.bash` setup script from zsh; it resolves its prefix using
Bash-specific variables and can produce misleading missing-file and Python
package errors.

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
ros2 run vixel_network vixel-network-setup -- \
  --machine-file /etc/vixel/machine.yaml validate
```

Provision the host services once on every new PC:

```bash
sudo env "PYTHONPATH=$PYTHONPATH" \
  "$(ros2 pkg prefix vixel_network)/lib/vixel_network/vixel-network-setup" \
  --machine-file /etc/vixel/machine.yaml install-service
```

This one command creates `vixel-network-setup.service` and
`vixel-ptp.service`, enables both, applies the network configuration, and starts
PTP immediately. Systemd starts them automatically on every subsequent boot;
you do not need to start PTP manually again. Re-run the command after moving the
workspace or changing which installation should own the services.

The installer also writes `/etc/linuxptp/vixel-ptp4l.conf`. This location is
readable by Ubuntu's packaged `ptp4l` AppArmor profile; do not move the generated
configuration under `/run/vixel` without adding an equivalent AppArmor rule.

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
