{
  description = "RAMM-only Vixel experiment with ROS 2 Lyrical";
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };
  outputs = { self, nixpkgs, nix-ros-overlay }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; overlays = [ nix-ros-overlay.overlays.default ]; };
      ros = pkgs.rosPackages.lyrical;
      rosEnv = ros.buildEnv {
        underlay = true;
        paths = with ros; [ ros-core ament-cmake ament-cmake-core ament-cmake-gtest ament-cmake-python
          rosidl-default-generators rosidl-default-runtime camera-info-manager cv-bridge
          rclcpp rclcpp-action rclpy sensor-msgs std-msgs geometry-msgs action-msgs
          builtin-interfaces tf2-ros tf2-ros-py launch launch-ros ros2launch ];
      };
    in {
      devShells.${system}.default = pkgs.mkShell {
        packages = [ rosEnv pkgs.colcon pkgs.cmake pkgs.ninja pkgs.pkg-config
          pkgs.aravis pkgs.glib pkgs.libxml2 pkgs.zlib pkgs.libusb1 pkgs.yaml-cpp pkgs.nlohmann_json pkgs.opencv
          pkgs.meson pkgs.git pkgs.nodejs pkgs.linuxptp pkgs.ethtool pkgs.iproute2
          pkgs.python3Packages.pytest pkgs.python3Packages.pyyaml pkgs.python3Packages.setuptools ];
        shellHook = ''
          export ROS_DOMAIN_ID=42
          export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
          export VIXEL_ROOT="$PWD"
          export PYTHONNOUSERSITE=1
          if [ -f "$VIXEL_ROOT/install/local_setup.bash" ]; then
            source "$VIXEL_ROOT/install/local_setup.bash"
          fi
        '';
      };
    };
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" "https://attic.iid.ciirc.cvut.cz/ros" ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q="
    ];
  };
}
