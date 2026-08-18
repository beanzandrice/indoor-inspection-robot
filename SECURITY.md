# Security Policy

## Supported versions

Security fixes are applied to the latest commit on `main`. No tagged releases are currently supported.

## Reporting a vulnerability

Use GitHub's private **Report a vulnerability** form in the repository Security tab. Do not disclose bridge framing, SSH routing, command forwarding, deployment, or robot-control vulnerabilities in a public issue before a fix is available.

Include the affected commit, reproduction conditions, impact, and a minimal sanitized example. Remove credentials, private addresses, real floorplans, personal paths, and identifiable imagery.

## Deployment assumptions

The TCP bridge is designed to run through authenticated SSH tunnels and loopback endpoints. Laptop ROS discovery also defaults to loopback through `LAPTOP_ROS_LOCALHOST_ONLY=1`, preventing unauthenticated LAN DDS peers from publishing goals into the command bridge. Do not disable that isolation without a trusted network or an authenticated SROS2 policy, and do not expose the TCP listeners directly to untrusted networks. Protect SSH keys, verify host keys, and restrict Pi and Go2 accounts to the access required by the project.

Treat relayed goals, initial poses, transforms, maps, images, and point clouds as sensitive operational data. Apply message-size limits, reject malformed frames, and stop robot operation when localization, TF, or command-channel health is uncertain.
