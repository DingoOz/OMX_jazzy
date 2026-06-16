# Errors & Defect Log for OMX_jazzy

This file records bugs, defects, incorrect patterns, and significant omissions discovered during development (per project CLAUDE.md guidelines). Each entry helps prevent recurrence.

### Documented custom services not implemented — 2026-04-XX (approx)

- **Severity:** Medium
- **Category:** API Misuse | Other
- **File(s):** `src/omx_control/omx_control/ik_controller.py`, `fk_controller.py`, `gripper_controller.py`; `src/omx_control/CMakeLists.txt`, `package.xml`; `README.md`, `docs/api.rst`
- **Pattern:** Declaring rosidl service interfaces + documenting a public ROS API (service types + expected names) while only implementing the topic side and leaving `create_service(...)` calls out of the Node `__init__` (and the handler methods unimplemented). The module docstrings even claimed "via service and topic interfaces".
- **Root cause:** The .srv files, CMake generation, package membership, and all user docs/CLAUDE.md were written with the intention of exposing `MoveToPosition` / `MoveJoints` / `GripperControl`, but the server-side plumbing in the three controller nodes was never added. Topic callbacks exercised the internal `move_*` methods; the service path was simply omitted.
- **Fix applied:** Added the three `from omx_control.srv import ...`, `create_service(..., '/omx/...', self._xxx_callback, callback_group=...)` registrations, the three thin `_callback` handlers that delegate to the pre-existing logic (respecting `wait_for_completion` and populating all response fields), and a tiny enhancement to `move_to_pose` return value (to supply `final_joint_positions` without duplicating IK computation or current-state queries). Updated the one affected internal call site. No changes to .srv, launches, or build.
- **Prevention rule:** When you add a .srv / .msg and update docs/README/CLAUDE.md claiming a new ROS interface, immediately add the corresponding `create_service` / `create_publisher` (or client) + minimal handler in the same change set. Grep the node source for the new type name before considering the feature "done". Treat "the docs say X works" as a test that must pass at review time. For hybrid ament_cmake + Python packages, verify the generated `from pkg.srv import Foo` succeeds after `colcon build --symlink-install` and that `ros2 service list` / `type` shows it when the node is launched.

(If more entries accumulate, add a ## Summary section at top grouping by category/root cause.)

## Summary (as of first entry)
- Primary recurring risk area: mismatch between advertised ROS API surface (topics/services/actions in docs + launch examples) and actual server/client code in the thin `omx_control` Python nodes.
- Recommendation: any future extension (new joints via the add_joint script, new controllers, camera features, etc.) should include an automated "does the documented interface exist at runtime?" check or at least an explicit service/topic list assertion in the verification steps.

### CI used --allow-overriding flag that was unrecognized in setup-ros env — 2026-06-10

- **Severity:** Medium
- **Category:** Configuration
- **File(s):** `.github/workflows/ci.yml`
- **Pattern:** Using `colcon build --symlink-install --allow-overriding <pkg>` (to prefer a vendored submodule copy over an apt-installed ROS package) directly in a GitHub Actions step after `ros-tooling/setup-ros` + a `rosdep install --from-paths` that did not skip the conflicting package.
- **Root cause:** `--allow-overriding` is registered by the `colcon-package-selection` extension. In the fresh environment created by setup-ros (which installs colcon-* debs + runs its own apt for dev tools) + rosdep pulling transitive depends like `dynamixel_sdk` for `dynamixel_hardware_interface`, the colcon parser in the "Build workspace" step did not accept the flag and errored with "unrecognized arguments". The flag works in a long-lived local dev workspace but not reliably in this minimal CI setup. The build step failed immediately; later steps were skipped.
- **Fix applied:** (1) Added `--skip-keys "dynamixel_sdk"` to the targeted rosdep command so the apt version is never installed. (2) Removed the `--allow-overriding dynamixel_sdk` line from the colcon build (the packages-select now builds the pure submodule copy). This matches the intent of the original README command while being robust for CI.
- **Prevention rule:** For packages that exist both as git submodules and as ros-*-<name> debs (DynamixelSDK family is a recurring case), always use rosdep `--skip-keys` in CI workflows instead of (or in addition to) relying on `--allow-overriding` at build time. Prefer testing the exact sequence (setup-ros + rosdep with the same --from-paths + colcon with the project's flags) when adding CI. Consider `ros-tooling/action-ros-ci` for future workflows as it is already used successfully by the vendored open_manipulator stack.

### LaTeX `\text` used without amsmath — 2026-06-16

- **Severity:** Low
- **Category:** Build
- **File(s):** `src/dynamixel_demo/docs/dynamixel_demo.tex`
- **Pattern:** Using math-mode macros that live in a specific package (`\text{...}` from `amsmath`, also `\eqref`, `\dfrac`, etc.) without `\usepackage{amsmath}`, producing a fatal "Undefined control sequence" with no PDF output.
- **Root cause:** The conversion formula used `\text{rad}`/`\text{tick}` inside `$...$`, but the preamble only loaded `geometry`, `listings`, `booktabs`, `enumitem`, `hyperref`, `xcolor` — not `amsmath`.
- **Fix applied:** Added `\usepackage{amsmath}` to the preamble; PDF now builds (4 pages).
- **Prevention rule:** Whenever a `.tex` uses `\text`, `\eqref`, `align`, `\dfrac` or other AMS macros, add `\usepackage{amsmath}` up front; do a clean `latexmk`/`pdflatex` pass before considering the doc done rather than assuming common macros are built-in.