# Changelog

## [1.1.1] - 2026-10-19

### Added
- Browser playground at https://megazron.github.io/zronos/: the real `zron` compiled to WebAssembly. Edit a robot, then boot it, use the shell or run doctor.

### Fixed
- The health summary said HEALTHY while the HAL was e-stopped or faulted; it now reports UNHEALTHY.

## [1.1.0] - 2026-10-19

### Added
- `zron up <manifest>`: real-time headless mode. The scheduler runs on the wall clock and SIGINT/SIGTERM halts it cleanly with a final health report. Pairs with `contrib/zronos@.service` for systemd.
- `make install` / `make uninstall` (`PREFIX`, `DESTDIR`); `make static`.
- An installed `zron` finds its MuJoCo bridge and default robot under `share/zronos`; `ZRON_BRIDGE` overrides the bridge.
- `tools/sync-chalao.sh`: vendor a Robot Chalao release; CI fails if `vendor/chalao` drifts from its pinned tag (`v1.0.0`).
- Release workflow: static Linux binary + install tree on every `v*` tag.
- MuJoCo bridge request validation and client verdict hardening (PR #1).

### Fixed
- The scheduler ticked every 0.5 s, so services faster than 2 Hz ran at 2 Hz. The tick now follows the fastest service (up to 200 Hz).
- Shell: `quit`/`exit`/`bye` and end-of-input now halt cleanly.
- Unknown subcommands are reported instead of being treated as a manifest path.

## [1.0.0] - 2026-09-17
Initial zronOS release: init, supervisor, scheduler, params, TF, RPC, health, record/replay, HAL + e-stop, shell, MuJoCo bridge (68/68 Menagerie models).
