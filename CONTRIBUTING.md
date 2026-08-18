# Contributing

Contributions are welcome when they preserve the project's deployment, privacy, and robot-safety boundaries.

## Development workflow

1. Create a focused branch from `main`.
2. Make the smallest change that solves the problem.
3. Run the hardware-independent checks below.
4. Test deployment or robot-facing behavior only in a controlled environment.
5. Open a pull request describing validation, hardware impact, and rollback behavior.

```bash
python3 -m pip install "numpy<2" PyYAML pytest
python3 -m compileall -q scripts go2_navigation isaac tests tools
python3 tools/validation/validate_repository.py
python3 -m pytest -q tests
find scripts go2_scripts -name '*.sh' -print0 | xargs -0 -n1 bash -n
find scripts go2_scripts -name '*.sh' -print0 | xargs -0 shellcheck --severity=error
```

Hosted CI does not include a Unitree Go2, Raspberry Pi relay, ROS navigation graph, or Isaac Sim. State clearly which portions were validated with real hardware.

## Robot and deployment safety

- Keep navigation and command testing supervised with a clear stopping method.
- Do not increase speed, acceleration, or footprint assumptions without recorded hardware validation.
- Preserve a known-good deployed version and verify a new build before switching to it.
- Exercise reconnect, stale-data, TF, localization, and rollback behavior for network or deployment changes.

## Security and privacy

- Never commit `config/bridge.env`, credentials, SSH keys, or site-specific network details.
- Replace real maps and imagery with synthetic or cropped examples in reports and documentation.
- Keep bridge endpoints behind the documented SSH tunnels.
- Report vulnerabilities using [SECURITY.md](SECURITY.md).

## Licensing

The repository owner has not yet selected a license for project-owned code. Discuss substantial contributions in an issue before investing work, because accepting a patch does not by itself grant downstream reuse rights. Preserve the notices described in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md), and do not remove or relicense upstream-derived files without confirming their terms.
