# TASKS

Deferred work for `e-axle-stand`. Written at the cleanup phase of session
`2026-08-26-A-e-axle-stand` (topic: *manual control session*).

No tracker is configured for this project (there is no `DEVELOPMENT.md` with a
`## Program Management` section), so per the program-management using
reference this file is the task tier. If a tracker is wired up later, these
entries move there.

Each entry carries: **Source** (where it came from), **Why deferred**, and
where useful a **TASK-NOTE** with the detail a future builder needs so the
reasoning is not re-derived from scratch.

Severity: `safety` > `correctness` > `reproducibility` > `hygiene`.

---

## T-1 — Wire interlock evaluation into `StandSession.tick()` `safety`

**Status:** closed. Wired in session `2026-09-02-A-interlock-safety`: Step 3
added `_evaluate_trip()`/`_evaluate_instantaneous()` and the trip action in
`tick()` (design.md §1-§5); Step 4 added `acknowledge_trip()` and `can_arm()`'s
"no active trip" gate (design.md §7-§8). All nine wired interlocks now trip
the stand in the fixed, first-wins order — see README.md's "Trip evaluation"
and "Recovery" sections for the behavior, not restated here. Interlock 11
stays deliberately unwired (Open question 3). The three loose ends below are
all resolved:

1. Resolved — the stale comment is gone; `can_arm()`'s "no active trip" term
   is implemented, not promised.
2. Resolved — `acknowledge_trip()` is the recovery path (README.md
   "Recovery").
3. Resolved — `zero_all()` is called from the trip action, with its internal
   order reversed per design.md §5.

`interlocks.py` implements thirteen trip evaluations and they are fully unit
tested (`tests/test_interlocks.py`, 30+ cases). **Nothing calls them.**
`session.py` imports only `LIMITS` from that module; no `check_*` function is
invoked anywhere in production code, `State.TRIPPED` is never assigned, and
there is no path that reaches it. The stand as built cannot trip.

**Source:** cleanup-phase final review (new finding — not raised by any earlier
review or gate).

**Why deferred:** no step in `plan.md` specified this wiring. Steps 0–6 covered
the interlock *functions* (Step 1) and the command/stop loop (Steps 2–5), and
the "Beyond this plan" section names only `main.py`/`constants.py`/
`app.connect`/the startup preflight as unbuilt. The wiring simply fell between
the two. Adding it now would be new feature work, not cleanup.

**TASK-NOTE.** Three loose ends the wiring must also close, all of which are
currently dangling:

1. `session.py:77-78` carries a comment promising a *"no active trip"* term
   would be added to `can_arm()` "once Step 1's interlocks are wired into
   `tick()` (Steps 3-5)". That term was never added. The comment is currently
   a false promise and should be resolved, not deleted.
2. There is no trip-recovery API. Once `TRIPPED` becomes reachable, nothing
   returns the session to `IDLE`: `disarm()` handles only `ARMED -> IDLE`
   (deliberately, per `session.py:85-88`), and there is no `reset()` /
   `clear_trip()`. Design the recovery path with the wiring, not after it.
3. `zero_all()` on `HardwareStand` already exists and is documented as
   "immediate trip-path zeroing" — it is the intended trip action and is
   likewise currently uncalled by any production code path.

---

## T-2 — Resolve the `InstroPSU` duck-typing shim in `stand.py` `correctness`

`HardwareStand.set_psu_output_enabled()` branches on
`hasattr(self._psu, "output_enable")`, and `get_telemetry()` branches on
`hasattr(measurement, "channel_data")`. Both branches exist to satisfy two
incompatible object shapes at once: the real Instro API, and the hand-rolled
`FakePsu`/`FakeController` doubles written in Steps 2–5.

**Source:** Step 6 of the build; verified at cleanup against the installed
branch. The real API has no `set_output_enabled` — `grep -rn set_output_enabled`
over the whole instro worktree returns nothing. The real method is
`InstroPSU.output_enable(enable: bool, channel: int, **kwargs) -> Command`
(`instro/psu/psu.py:287`). `plan.md`'s Step 6 sketch assumed the wrong name,
and the doubles in `test_session.py`, `test_speed_commanding.py`,
`test_brake_commanding.py` and `test_stop_sequencing.py` were built to that
assumption.

**Why deferred:** Step 6's choice was between breaking 22 already-green unit
tests or leaving the feature test red. The shim was the compromise that kept
both green, and reversing it is a test-refactor across four files — real work
with a real regression surface, not a cleanup edit.

**TASK-NOTE.** Recommended resolution: update the four test doubles to mirror
the real shapes (`output_enable(enable, channel)` on `FakePsu`; a
`channel_data`-bearing measurement or `None` on `FakeController`), then delete
both `hasattr` branches. Rationale: a test double whose shape does not match
the real object is the exact defect that produced this shim, and leaving the
shim in place preserves the ability for the doubles to drift again silently.
The counter-argument (that the shim is cheap and the doubles are deliberately
minimal) is real but weaker for a safety-relevant adapter.

---

## T-3 — `channel=1` is a magic literal in `set_psu_output_enabled()` `hygiene`

`stand.py:139` hardcodes `channel=1`. It is *correct* — this stand's PSU is
constructed with `num_channels=1` in both `main.py`-to-be and the feature
test's `build_stand()`, so channel 1 is the only channel that exists — and the
reasoning is captured in the surrounding comment. It is still an unnamed
constant in the one call that energises the machine.

**Why deferred:** folds naturally into T-2 (same method, same edit) and into
the `constants.py` work in T-11. Doing it standalone means touching that method
three times.

---

## T-4 — Decide brake deceleration limiting: instant clamp vs. ramp `safety`

`StandSession.tick()` applies the brake setpoint **instantly** (clamped to
`LIMITS.max_brake_a`, no ramp) while `ARMED`/`RUNNING`. `LIMITS.
brake_ramp_a_per_s` (2.0 A/s) is used only on the way *down*, in the STOP
sequence.

**This is an unresolved contradiction inside `plan.md`, not a closed
decision.** Plan Step 4 specifies instant-clamp and its test asserts the limit
is reached in a single tick. But `plan.md`'s own "Risks and open questions"
section, item 2, carries a human comment on exactly this point:

> *"I think we should try to put limits on deceleration here. We certainly
> don't want to hammer the driveline."*

The build implemented the plan's literal spec (per the standing instruction not
to re-litigate the plan mid-build), which means the human's comment was never
answered. **Step 4 did not close this; it went the other way.**

**Why deferred:** it is a human decision about machine behaviour, not a defect
to fix unilaterally. It also interacts directly with T-5.

**TASK-NOTE.** If a symmetric ramp-up is adopted, `LIMITS.brake_ramp_a_per_s`
already exists and the `_ramp_toward` helper is already there and tested — the
change is small. The test that would need rewriting is the Step 4 case
asserting single-tick attainment of the clamp.

---

## T-5 — No speed frame is sent during the STOPPING brake bleed-off `safety`

**Status:** shipped, pending hardware confirmation. Fixed in session
`2026-09-02-A-interlock-safety` (Step 1 of that session's plan.md, per
design.md §9): `session.py`'s `commanded_brake_a > 0.0` branch now re-sends
the unchanged `commanded_erpm` every STOPPING tick, after that tick's brake
frame. `tests/test_trip_and_recovery.py` Act 1 hard-asserts the new behaviour
(a speed frame is sent on every such tick, unchanged, and the largest gap
between speed frames anywhere in the STOP window is ≤ 200 ms). This is
**not** "closed" — the fix is software-verified only; the underlying premise
(a VESC FSESC 75200 in speed control tolerates an unchanged `SET_RPM` while
its regen load bleeds off) is unverified on real hardware. See the
hardware-caveat comment at the changed branch in `session.py` and design.md's
§9 boxed caveat for what tomorrow's bench run must confirm, and for what
reverting costs if it doesn't hold up.

**Source:** cleanup-phase final review (new finding).

In `session.py:172-182`, while `state is STOPPING` and `commanded_brake_a >
0.0`, the branch is a bare `pass`: the brake frame is sent, and **no speed
frame is sent at all**. The comment calls this "speed is frozen."

The concern: `design.md` records a **1000 ms firmware command watchdog**, and
`session.py:122-131` states plainly that the unconditional per-tick speed
resend "is the whole safety mechanism here" — Instro has no periodic transmit,
so that call is the only thing feeding the watchdog. During a STOP from full
brake, `LIMITS.brake_ramp_a_per_s` is 2.0 A/s, so bleeding 20 A off takes ~10 s.
For ~9 of those seconds no speed frame goes out, the drive motor's watchdog
expires at ~1 s, and the firmware releases the drive while the absorbers are
still regen-braking — which is the inverse of what interlock 13
("Load off first, then speed. Never the other way round.") is meant to
guarantee.

**Why deferred:** the code matches the plan's Step 5 spec exactly, and its
tests assert that spec. Whether the watchdog release during bleed-off is a
benign coast-down (`design.md:413` treats watchdog expiry as the fail-safe) or
a defeat of the stop ordering is a machine-behaviour judgement that needs the
design author and, realistically, the hardware. **It is invisible to the
current test suite** — the feature test's plant is obedient and never models
the watchdog.

**TASK-NOTE.** The proposed fix — in the `commanded_brake_a > 0.0` branch,
re-send the *unchanged* `commanded_erpm` rather than sending nothing — is what
was adopted and shipped (see Status above). It holds the setpoint without
ramping it, preserving both the ordering invariant and the watchdog feed.
Hardware confirmation is still outstanding; do not treat this TASK-NOTE's
original "verify before adopting" as satisfied by the software fix alone.

---

## T-6 — Cutting PSU output mid-run neither stops nor disarms the session `safety`

**Status:** shipped, pending hardware confirmation. Fixed in session
`2026-09-02-A-interlock-safety` (Step 5 of that session's plan.md, per
design.md §10/§10a): `StandSession.set_psu_output_enabled(False)` now forces
Step 3's trip action (`Trip(reason="psu_disabled", node=None)`) while
ARMED/RUNNING/STOPPING, and — in every state, including IDLE/TRIPPED — takes
the bus to 0V via the new `HardwareStand.zero_bus_voltage()` rather than ever
calling `output_enable(False)`. `set_psu_output_enabled(True)` is unchanged.
`tests/test_trip_and_recovery.py` Act 3 hard-asserts this (no `OUTP...OFF`
write, a `VOLT 0.000` write, machine zeroed first). This is **not** "closed" —
the physical premise (do the three VESC controllers stay powered once the bus
is commanded to 0V?) is unverified on real hardware; see design.md §10's
boxed caveat and README.md's "T-6" section for what tomorrow's bench check
must confirm.

**Source:** cleanup-phase final review (original finding).

`StandSession.set_psu_output_enabled(False)` de-energised the supply and set
`self._psu_enabled = False`, but performed no state transition. If the operator
killed the PSU while `RUNNING`, the session stayed `RUNNING` and `tick()` kept
ramping and transmitting speed and brake frames to an unpowered machine.
`_psu_enabled` gated only `can_arm()`, which gated only `arm()` — a state the
session had already left.

**Why originally deferred:** new finding, and the correct reaction (force
`STOPPING`? trip? refuse the call while active?) was a safety-policy decision
for a human, and was most naturally decided alongside T-1's trip wiring. Both
have now landed together in session `2026-09-02-A-interlock-safety`.

---

## T-7 — Declare the project's dependencies; pin/record the Instro branch `reproducibility`

This project has **no dependency declaration of any kind** — no
`pyproject.toml`, no `requirements.txt`, no lockfile. The test suite passes
only because `.venv` happens to contain an editable install of an unmerged
branch:

- `instro` 1.10.0 and `instro_unstable` 1.5.0, both `editable: true`
- installed from `/Users/william/Projects/nominal/instro-vesc6-worktree`
- branch `issue-362-vesc6-motor-controller-driver`
- **commit `2c0baa73edbde6a4592b4e9285881e78ddcd3b43`**

This closes **design-review finding R4-1** ("record which commit of the branch
was installed") — the SHA is recorded here rather than being lost in the Step 6
dispatch transcript.

**The worktree at `/Users/william/Projects/nominal/instro-vesc6-worktree` is a
local development-environment aid, not a deliverable of this project.** It is
outside this repository, is not referenced by any tracked file, and will not
exist on any other machine. A fresh clone of this repo cannot run
`tests/test_manual_control_session.py` today.

`design.md`'s rev-3 dependency plan already decided the answer — "its author
validates and releases it, and **this repo declares a version floor**" — that
decision was simply never implemented, because no step in `plan.md` owned it.

**TASK-NOTE.** Do this when `instro#386` lands: replace the branch install with
a released version floor in a real `pyproject.toml`. Until then, record the
branch + SHA install procedure somewhere tracked (a README, or the
`pyproject.toml`'s comments) so the environment is reproducible by someone who
was not in this session. `plan.md` has the exact `uv pip install -e` incantation
and the note on why a plain `pip install git+...` is a trap.

---

## T-8 — End-to-end fault-reaction test against a diverging plant `correctness`

**Source:** design rev-4 review finding **R4-2**, carried forward by `plan.md`
risk item 3, and confirmed still open at cleanup.

The feature test's simulated plant answers exactly what was commanded and never
diverges, so the suite proves correct *commanding* and correct *stopping* but
proves nothing about fault *reaction*. This is explicitly out of scope for this
feature — the feature test's own docstring says so — and each of the thirteen
trip evaluations does have unit coverage, so it is not a total gap.

It becomes materially more important once **T-1** lands: today there is no
fault-reaction behaviour to test, which is itself the reason this test cannot
be written yet. **T-8 depends on T-1.**

The human's recorded answer on `plan.md` risk item 3 was *"We will test this on
real hardware. That's fine."* — that remains a legitimate answer for the demo,
but it is not a substitute for a regression test once the wiring exists.

---

## T-9 — Differential speed-balance trim loop (v2) `deferred-scope`

**Source:** `design.md`, "Deferred technical decisions" (Theory §6.4).
Deferred on measured evidence, not cost: the MVP runs open-loop brake current
to both absorbers with no trim of any kind. Revisit if spread on the assembled
stand behaves differently from commissioning. The `spread` interlock
(`check_spread`, 15% above a 500 ERPM floor) is the safety net in the meantime.
Confirmed still deferred; not forgotten.

---

## T-10 — VESC fault-code interlock: the one specified interlock not built `deferred-scope`

**Source:** `design.md`, "The one specified interlock that is not in the table
— a called-out gap." Theory §9 #5 requires controlled shutdown on any non-zero
VESC fault code. No frame this stand receives carries one: `STATUS_1/4/5` have
no fault field, `STATUS_6` is discarded, and `VESC6._parse_status_frame`
returns `{}` for anything else. `MotorTelemetry` has no fault field, and
instro PR #386 explicitly defers the fault surface.

Partly covered by three proxies (dropout catches loss of torque in 0.3 s;
staleness catches loss of broadcast in 0.5 s; VESC hardware overvoltage latches
in the controller). **Not covered:** a fault that leaves the node both
broadcasting and delivering torque.

`design.md` recommends confirming with Tyler whether the VESC CAN status set
can include a fault field, and notes that a fault-code decode contributed
upstream to `VESC6` is worth more than the same code written privately here —
this stand is the natural rig to exercise it. Confirmed still deferred.

---

## T-11 — Composition root: `main.py`, `constants.py`, `app.connect`, startup preflight `feature`

**Source:** `plan.md`, "Beyond this plan." Deliberately not a step, because the
feature test never imports `main`. Still unbuilt:

- `main.py` — builds the real `CanDriver`/`VESC6`/`InstroMotorController`/
  `InstroPSU` graph and injects it into `HardwareStand` (the feature test's
  `build_stand()` is the working template).
- `constants.py` — the machine-dependent values (also the home for T-3's
  channel number).
- `app.connect` — currently a **0-byte untracked placeholder**. See T-13.
- The startup preflight: `import instro.unstable.motorcontroller`, refuse to
  arm with a named message on `ImportError`.

**TASK-NOTE.** `plan.md` requires the `create-connect-app` skill
(`/Users/william/Projects/nominal/connect/.claude/agents/create-connect-app.md`)
to be loaded *before* writing any `app.connect` YAML or `connect_python` code.
Do not start this task without it.

---

## T-12 — `interlocks` collides with an installed PyPI package `reproducibility`

**Source:** cleanup-phase final review (new finding).

This project's `.venv` contains a third-party PyPI package literally named
`interlocks` (v0.2.1, a Python quality-tooling runner), which collides with
this project's top-level `interlocks.py` — the module that holds every
commissioned safety limit.

The local module wins today only because pytest puts the project root ahead of
site-packages on `sys.path`. Verified: running the interpreter from any other
working directory, `import interlocks` resolves to
`site-packages/interlocks/__init__.py`, and `interlocks.LIMITS` does not exist.

**Why deferred:** the fix is either `uv pip uninstall interlocks` (if nothing
needs it — it appears to be an unrelated dev tool that was installed into this
venv) or renaming the module / adopting a package layout (`e_axle_stand/`),
which is a structural change unsuitable for a cleanup pass and which would
touch every import in every test file.

**TASK-NOTE.** This becomes an outright breakage the moment this project is
packaged, installed, or invoked as a console script rather than run from its
own root. For a module carrying safety limits, a silent resolution to the wrong
module is the worst available failure mode. Prefer the package layout.

---

## T-13 — Decide the fate of `reference/` and the empty `app.connect` `hygiene`

Resolved at cleanup: `.gitignore` was written (it was previously empty) and now
excludes `__pycache__/`, `*.py[cod]`, `.venv/`, `.pytest_cache/`, `.DS_Store`,
the `.ailly/` session folders (with `.ailly/developer/TASKS.md` explicitly
re-included so this file is tracked), and
`reference/Tyler Software/logs/`. That took `git status` from 2361 untracked
files to 5.

**Two decisions deliberately left to a human, and left visible in
`git status` rather than hidden by an ignore rule:**

1. **`reference/Tyler Software/{comms_check,dut_step_test,dyno_test}.py`** —
   Tyler's commissioning scripts, ~92 KB total. These are the prior art
   `design.md` was written against and are genuinely valuable to keep with the
   code. They are also someone else's software; provenance and licensing are a
   human call. *Not* ignored, so the decision stays in front of you. (The
   79 MB of CSV logs beside them, and the checked-in Windows `.venv` with
   `.exe` binaries, are ignored — 111 MB of CAN logs does not belong in git
   under any answer to this question.)
2. **`app.connect`** — a 0-byte placeholder. It should either be written (see
   T-11) or deleted; committing an empty file is the one option with no
   upside.

---

## T-14 — Adopt an explicit formatter / lint configuration `hygiene`

The project has no `pyproject.toml`, `ruff.toml`, or any other tool config, so
it has no declared formatting convention.

Done at cleanup: `ruff check` reported 4 `F401` unused-import findings, all in
tests where the module-level import is deliberate (an implicit "this module
imports cleanly" smoke check). Fixed by widening the existing `# noqa: E402`
to `# noqa: E402,F401` in `tests/test_api_surface.py` and
`tests/test_interlocks_units_import_guard.py` — preserving the imports rather
than deleting them, since deleting them would delete the check. `ruff check`
is now clean across all sources and tests.

**Not done, deliberately:** `ruff format` would reformat 9 of the 13 staged
files (line-wrapping of long assert messages and multi-line call arguments).
Applying it was declined because (a) with no config file, ruff's defaults are
an unadopted default rather than this project's convention, and (b) it would
put pure-whitespace churn across two thirds of the diff immediately before a
human reviews the repository's first-ever commit.

**TASK-NOTE.** If a formatter is wanted, the moment to run it is right after
that first commit lands, so the reformat is one clean isolated commit:
`ruff format interlocks.py units.py session.py stand.py tests/`. Note that the
sibling `instro` repo enforces `ruff format` + `ruff check` + `mypy` via
`just check`; matching that convention here would be consistent with the rest
of the org.

---

## T-15 — `tick_hz` is stored but never read `hygiene`

`StandSession.__init__` takes a required `tick_hz` and assigns
`self.tick_hz`; nothing ever reads it. The real cadence comes from the `dt`
argument to `tick()`. A caller can construct the session claiming 50 Hz and
then tick with a mismatched `dt` and nothing will notice.

It is part of the asserted API surface (`tests/test_api_surface.py:229`), so
removing it is an API change. Either use it (e.g. to validate `dt`, or to
supply a default `dt`) or drop it — but not silently.

---

## T-16 — Remove the session folder after the squash-merge is approved `process`

The cleanup phase reference specifies removing
`.ailly/developer/2026-08-26-A-e-axle-stand/` at the end of cleanup. **This was
deliberately not done**, and the reason should be checked before someone does
it:

This repository has **zero commits**. `.ailly/` is untracked. Deleting that
folder now would permanently and unrecoverably destroy `research.md`,
`design.md` (116 KB, four revisions), `plan.md` (47 KB, three revisions), the
nine review documents and the map — the entire decision record behind the code
— *before* the human approval gate that decides whether the work is accepted at
all. The reference assumes a repository where that history is already committed
somewhere; here it is not.

**Do this only after the squash-merge is approved and committed.** At that
point the folder has served its purpose and the deferred record lives here.

---
