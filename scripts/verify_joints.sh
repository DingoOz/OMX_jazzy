#!/usr/bin/env bash
# Cross-check arm joint names across every layer of the OMX_jazzy stack.
#
# Exits non-zero if any pair of layers disagrees. Intended to be run after
# editing the URDF / ros2_control / MoveIt configs (and especially after
# running scripts/add_joint.py).

set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DESC="$REPO/src/open_manipulator/open_manipulator_description"
BRINGUP="$REPO/src/open_manipulator/open_manipulator_bringup/config/open_manipulator_x"
MOVEIT="$REPO/src/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x"
OMX="$REPO/src/omx_control"

fail=0

# ---- helpers ---------------------------------------------------------------

# Print one joint name per line, sorted, deduped.
normalize() { tr -d '[:space:]' | tr ',' '\n' | grep -v '^$' | sort -u; }

# Compare two lists, report missing/extra.
compare() {
    local label_a="$1" list_a="$2" label_b="$3" list_b="$4"
    local missing extra
    missing="$(comm -23 <(printf '%s' "$list_a") <(printf '%s' "$list_b"))"
    extra="$(comm -13   <(printf '%s' "$list_a") <(printf '%s' "$list_b"))"
    if [[ -z "$missing" && -z "$extra" ]]; then
        echo "  ok   $label_a == $label_b"
    else
        fail=1
        echo "  FAIL $label_a != $label_b"
        [[ -n "$missing" ]] && echo "       in $label_a but not $label_b: $(echo "$missing" | tr '\n' ' ')"
        [[ -n "$extra"   ]] && echo "       in $label_b but not $label_a: $(echo "$extra"   | tr '\n' ' ')"
    fi
}

# ---- extract joint lists from each layer -----------------------------------

# URDF: all revolute/prismatic joints (excludes fixed end_effector_joint).
urdf_all="$(
    grep -oE '<joint name="\$\{prefix\}[^"]+" type="(revolute|prismatic)"' \
        "$DESC/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro" \
        | sed -E 's/.*\$\{prefix\}([^"]+)" type=.*/\1/' \
        | sort -u
)"
# Arm-only set (the arm-controller layers don't include grippers).
urdf_joints="$(echo "$urdf_all" | grep -vE '^gripper_')"
# Gazebo includes both arm joints and both gripper joints (incl. mimic).
urdf_with_grippers="$urdf_all"

# ros2_control arm joints (non-mimic, i.e. without the sim-only gripper_right).
ros2_control_joints="$(
    awk '
        /<xacro:if value="\$\(arg use_sim\)">/ {in_sim=1}
        /<\/xacro:if>/                          {in_sim=0; next}
        in_sim                                  {next}
        /<joint name="\$\{prefix\}/ {
            match($0, /<joint name="\$\{prefix\}([^"]+)"/, m)
            print m[1]
        }
    ' "$DESC/ros2_control/open_manipulator_x_position.ros2_control.xacro" \
    | sort -u
)"

# Gazebo SimpleTransmission joints (non-mimic — sim duplicates gripper_right_joint).
gazebo_joints="$(
    grep -oE 'SimpleTransmission [^>]*joint="\$\{prefix\}[^"]+"' \
        "$DESC/gazebo/open_manipulator_x.gazebo.xacro" \
        | sed -E 's/.*\$\{prefix\}([^"]+)".*/\1/' \
        | sort -u
)"

# Bringup arm_controller.joints
hwcm_joints="$(
    awk '
        /^[[:space:]]*arm_controller:/ {in_arm=1; next}
        in_arm && /^[[:space:]]*joints:/ {in_joints=1; next}
        in_joints && /^[[:space:]]*-[[:space:]]+/ {gsub(/[[:space:]-]/,""); print; next}
        in_joints && !/^[[:space:]]*-/ {in_joints=0; in_arm=0}
    ' "$BRINGUP/hardware_controller_manager.yaml" \
    | sort -u
)"

# Initial positions joint_names
initial_joints="$(
    awk '
        /^[[:space:]]*joint_names:/ {in_jn=1; next}
        in_jn && /^[[:space:]]*-[[:space:]]+/ {gsub(/[[:space:]-]/,""); print; next}
        in_jn && !/^[[:space:]]*-/ {in_jn=0}
    ' "$BRINGUP/initial_positions.yaml" \
    | sort -u
)"

# Initial positions home list length (for size check).
home_len="$(
    awk -F'[][,]' '/^[[:space:]]*home:/ {
        n=0; for (i=2;i<NF;i++) if ($i ~ /[^[:space:]]/) n++; print n; exit
    }' "$BRINGUP/initial_positions.yaml"
)"

# MoveIt SRDF arm group (filter out end_effector_joint which is fixed).
srdf_joints="$(
    awk '
        /<group name="arm">/ {in_arm=1; next}
        /<\/group>/          {in_arm=0}
        in_arm && /<joint name=/ {
            match($0, /<joint name="([^"]+)"/, m)
            if (m[1] != "end_effector_joint") print m[1]
        }
    ' "$MOVEIT/open_manipulator_x.srdf" \
    | sort -u
)"

# MoveIt joint_limits.yaml keys (includes grippers).
mlimits_joints="$(
    awk '
        /^joint_limits:/ {in_jl=1; next}
        in_jl && /^[[:space:]]{2}[^[:space:]]/ {
            gsub(/:.*/,"")
            gsub(/[[:space:]]/,"")
            print
        }
    ' "$MOVEIT/joint_limits.yaml" \
    | sort -u
)"

# MoveIt controllers arm_controller.joints
mctrl_joints="$(
    awk '
        /arm_controller:/ {in_arm=1; depth=0; next}
        in_arm && /joints:/ {in_joints=1; next}
        in_joints && /^[[:space:]]*-[[:space:]]+/ {gsub(/[[:space:]-]/,""); print; next}
        in_joints && !/^[[:space:]]*-/ {in_joints=0; in_arm=0}
    ' "$MOVEIT/moveit_controllers.yaml" \
    | sort -u
)"

# omx_control control_params.yaml joint_limits keys
cp_joints="$(
    awk '
        /^[[:space:]]*joint_limits:/ {in_jl=1; depth=0; next}
        in_jl && /^[[:space:]]{6}[^[:space:]]/ {
            line=$0; gsub(/:.*/,"",line); gsub(/[[:space:]]/,"",line); print line
        }
        in_jl && /^[^[:space:]]/ {in_jl=0}
    ' "$OMX/config/control_params.yaml" \
    | sort -u
)"

# omx_control Python ARM_JOINTS (from fk_controller; treat as canonical Python list).
py_joints="$(
    grep -oE "ARM_JOINTS\s*=\s*\[[^]]+\]" "$OMX/omx_control/fk_controller.py" \
        | head -1 \
        | grep -oE "'[^']+'" \
        | tr -d "'" \
        | sort -u
)"

# Sanity-check the same list in ik_controller and teach_playback.
py_ik="$(
    grep -oE "ARM_JOINTS\s*=\s*\[[^]]+\]" "$OMX/omx_control/ik_controller.py" \
        | head -1 \
        | grep -oE "'[^']+'" \
        | tr -d "'" \
        | sort -u
)"
py_tp="$(
    grep -oE "ARM_JOINTS\s*=\s*\[[^]]+\]" "$OMX/omx_control/teach_playback.py" \
        | head -1 \
        | grep -oE "'[^']+'" \
        | tr -d "'" \
        | sort -u
)"

# ---- report ----------------------------------------------------------------

echo "URDF actuated joints (excluding fixed):"
echo "$urdf_with_grippers" | sed 's/^/  /'
echo "Arm-only set (used for cross-layer checks):"
echo "$urdf_joints" | sed 's/^/  /'
echo

# ros2_control xacro: arm joints + gripper_left_joint (hardware path).
urdf_ros2_control="$(echo "$urdf_with_grippers" | grep -v '^gripper_right_joint$')"

echo "Layer comparisons:"
compare "URDF (arm+left grip)"  "$urdf_ros2_control" \
        "ros2_control"          "$ros2_control_joints"
compare "URDF (all actuated)"   "$urdf_with_grippers" \
        "gazebo transmissions"  "$gazebo_joints"
compare "URDF (arm)"            "$urdf_joints" \
        "bringup arm_controller" "$hwcm_joints"
compare "URDF (arm)"            "$urdf_joints" \
        "MoveIt SRDF arm"       "$srdf_joints"
# MoveIt joint_limits.yaml carries both arm joints and grippers.
compare "URDF (all actuated)"   "$urdf_with_grippers" \
        "MoveIt joint_limits"   "$mlimits_joints"
compare "URDF (arm)"            "$urdf_joints" \
        "MoveIt controllers"    "$mctrl_joints"
compare "URDF (arm)"            "$urdf_joints" \
        "omx_control yaml"      "$cp_joints"
compare "URDF (arm)"            "$urdf_joints" \
        "fk_controller.py"      "$py_joints"
compare "fk_controller.py"      "$py_joints" \
        "ik_controller.py"      "$py_ik"
compare "fk_controller.py"      "$py_joints" \
        "teach_playback.py"     "$py_tp"

# Initial-position list size vs. joint count.
echo
arm_only_count="$(echo "$urdf_joints" | grep -vE '^gripper_' | wc -l)"
echo "Initial-positions home list size check:"
if [[ -n "$home_len" && "$home_len" -eq "$arm_only_count" ]]; then
    echo "  ok   home list has $home_len entries for $arm_only_count arm joints"
else
    fail=1
    echo "  FAIL home list has ${home_len:-?} entries but URDF declares $arm_only_count arm joints"
fi

echo
if [[ $fail -eq 0 ]]; then
    echo "All layers agree."
else
    echo "Inconsistencies detected — fix the layers above before launching." >&2
fi
exit $fail
