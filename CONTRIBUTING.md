# Contributing

## The rule that matters

Nothing in this repository asserts meaning it cannot justify. That is inherited from the
upstream protocol project and it is why, for example, the board's deprecated one-byte
error code is reported as a number while `SYSTEM_ERROR` is reported by name: the
specification names the bits of one and not the other.

Concretely:

- A byte layout goes in only with a citation — the vendor specification, or a worked
  example whose bytes are asserted in a test.
- A field that has not been confirmed against hardware is either not published, or
  published with the caveat attached where a consumer will see it. The `sensor_msgs/Imu`
  vectors are the standing example.
- Do not describe a software interlock, watchdog, travel limit or stop command as a safety
  guarantee.

## Changes that can move a gimbal

Anything that can transmit a motion command needs a test that would fail without it.
Prefer a `GimbalCore` unit test asserting the wire-level frame; reach for the integration
suite when the property only exists once a node is running.

A useful check when adding a safety rule: revert just the fix and confirm the new test
actually fails. Several tests here were written that way, and one of them turned out to
pass against the bug it was meant to catch until its fixture was corrected.

## The vendored protocol library

`vendor/simplebgc32-control` is a submodule, not a copy. Protocol changes belong upstream;
bump the submodule here afterwards. Upstream's own byte-level suite runs as part of this
build precisely so a bump that changes the wire format fails here.

## Before opening a pull request

```bash
make test
```

Everything must be green, linters included. If you claim hardware support, say which board
revision, firmware version and serial adapter you tested with, and whether a payload was
fitted.
