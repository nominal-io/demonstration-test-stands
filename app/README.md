# Post Testing Notes

Running doesn't *just* work. A few things are necessary first.

1. `just build` builds the `e-axle` wheel into the proper location
1. `instro` (at the time of writing) hasn't had a new release with Will's PSB changes, thus needs a custom wheel built from source and included in the `app/wheels`
	a. This requires a version bump locally and a `just build` in instro
1. `gs_usb` (at the time of writing) hasn't had a release with the fixes for MacOS, and requires the same kind of fix as `instro`
	a. Similarly, this requires bumping its version and building locally with `uv build`

Once all wheels have been included you can confirm their inclusion in the debug logs in connect's python venv side panel.
