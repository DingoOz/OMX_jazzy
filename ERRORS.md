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