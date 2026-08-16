# Vendored SimpleBGC32 protocol sources

These files are copied verbatim from
[simplebgc32-control](https://github.com/magdang/simplebgc32-control), MIT licensed
(see `LICENSE.upstream`).

**Pinned at commit `7d2ab74f05ab38e83771c3e9acb6c45d10091adf`.**

They are copied rather than referenced as a git submodule so that this repository is
self-contained: clone it and build, with no second repository to fetch and no way to end
up with an empty directory. The cost is that upstream fixes do not arrive on their own,
which is why the commit is recorded above and why `sync.sh` exists.

## What is here, and what is deliberately not

| File | Why |
|---|---|
| `src/sbgc_api.c`, `include/sbgc_api.h` | The wire protocol: framing, checksums, control frames, angle decoding |
| `src/sbgc_params.c`, `include/sbgc_params.h` | Telemetry, profile and board-info decoders |
| `src/sbgc_gui_config.c`, `include/sbgc_gui_config.h` | Serial port discovery |
| `test/test_sbgc_api.c` | Upstream's 37 byte-level assertions against BaseCam's published examples |
| `test/sbgc_sim.py` | A pty-backed board simulator, used by the driver's integration tests |

Upstream's `httpd.c`, `gamepad.c`, its two applications and its web UI are **not** here.
They belong to the standalone tools and have no place in a ROS driver.

## Do not edit these in place

A protocol change belongs upstream, where its own test suite covers it. Editing here
forks the wire format silently, which is the exact failure this project already had once:
a second, drifting reimplementation in another language.

To take a newer upstream revision:

```bash
./sbgc_protocol/vendor/sync.sh                 # latest main
./sbgc_protocol/vendor/sync.sh <commit-ish>    # a specific revision
```

Then run `make test`. Upstream's own byte-level suite runs as part of this build, so a
revision that changes the wire format fails here rather than on a gimbal.
