# Security policy

## Reporting a vulnerability

Please report suspected vulnerabilities through GitHub private vulnerability
reporting for this repository. Do not include credentials, private network
details, or camera images in a public issue.

## Deployment model

Vixel is experimental and assumes a trusted robot or sensor computer:

- The web gateway has no built-in authentication and binds to `127.0.0.1` by
  default. Access it through SSH or an authenticated reverse proxy.
- ROS 2 graph access is not secured by Vixel. Use SROS 2 and network isolation
  when untrusted hosts can reach the ROS domain.
- `vixel-network-setup` changes NetworkManager profiles, sysctls, and NIC tuning.
  Run its dry-run first and grant root access only to reviewed configuration.
- Only mark dedicated sensor networks as `approved`. Discovery on other
  interfaces is informational and must never trigger provisioning.

No released version currently receives long-term security support. Security
fixes will be documented in the changelog and GitHub releases.
