local make = oslo.make

-- Invoke ROS tools in this project's Nix environment; no shell wrapper needed.
local function ros(...)
  local argv = { "nix", "develop", "path:.", "--accept-flake-config", "--command" }
  for _, value in ipairs({ ... }) do argv[#argv + 1] = value end
  return oslo.run(argv)
end

make.recipe {
  name = "build",
  desc = "Build Vixel with ROS 2 Lyrical",
  run = function()
    assert(ros("colcon", "build", "--symlink-install", "--parallel-workers", "2",
      "--cmake-args", "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=ON").ok,
      "Vixel build failed")
  end,
}

make.recipe {
  name = "test",
  desc = "Run Vixel tests and show their results",
  run = function()
    local tests = ros("colcon", "test", "--return-code-on-test-failure")
    local results = ros("colcon", "test-result", "--verbose")
    assert(tests.ok and results.ok, "Vixel tests failed; see the results above")
  end,
}

make.recipe {
  name = "run",
  desc = "Launch Vixel and its dashboard on localhost:8080",
  run = function()
    local vision = oslo.run {
      "systemctl", "--user", "is-active", "--quiet", "smartweg-vision.service",
      capture = true,
    }
    assert(not vision.ok,
      "Stop the camera GUI first: systemctl --user stop smartweg-vision.service")
    local root = oslo.sys.pwd()
    assert(ros("ros2", "launch", "vixel", "cameras_launch.py",
      "machine_file:=" .. root .. "/config/machine.yaml",
      "inventory_file:=" .. root .. "/state/inventory.yaml",
      "web_preview:=true", "web_port:=8080").ok, "Vixel launch failed")
  end,
}
