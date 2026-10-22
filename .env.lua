-- This experiment stays on RAMM; it has no sync hooks.
local root = oslo.direnv.dir() or oslo.sys.pwd()
oslo.direnv.nix_develop { flake = "path:" .. root }
