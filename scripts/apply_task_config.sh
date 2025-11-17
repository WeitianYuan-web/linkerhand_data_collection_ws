#!/usr/bin/env bash
set -euo pipefail

# Determine repository root (one level up from this script directory)
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

JSON_PATH="${ROOT_DIR}/task_config.json"
# 使用 linkerhand_cl 替代 linker_hand_ros2_sdk
LH_LAUNCH_SINGLE="${ROOT_DIR}/src/linkerhand_cl/launch/linker_hand.launch.py"
LH_LAUNCH_DOUBLE="${ROOT_DIR}/src/linkerhand_cl/launch/linker_hand_double.launch.py"
GUI_LAUNCH="${ROOT_DIR}/src/linker_hand_ros2_sdk/gui_control/launch/gui_control.launch.py"  # GUI工具仍使用原SDK
# linker_telop_sdk 已移除，不再需要 base_config.yml
# setting.yaml 已弃用，linkerhand_cl 使用 task_config.json
# 保留此路径仅用于向后兼容
SETTING_YAML="${ROOT_DIR}/src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml"

if [[ ! -f "${JSON_PATH}" ]]; then
  echo "task_config.json not found at: ${JSON_PATH}" >&2
  exit 1
fi

# Read JSON using Python and export as shell variables safely
eval "$(JSON_PATH="${JSON_PATH}" python3 - <<'PY'
import json, os, shlex
p = os.environ['JSON_PATH']
with open(p, 'r', encoding='utf-8') as f:
    d = json.load(f)

def emit(k, v):
    if isinstance(v, bool):
        print(f"{k}={'true' if v else 'false'}")
    else:
        print(f"{k}={shlex.quote(str(v))}")

emit('ARM_TYPE', d.get('armType', ''))
emit('HAND_SIDE', d.get('handSide', ''))
emit('HAND_MODEL', d.get('handModel', ''))
emit('COLLECT_TACTILE', d.get('collectTactile', False))
emit('CAM_D455', d.get('cameraD455Count', 0))
emit('CAM_D405', d.get('cameraD405Count', 0))
PY
)"

# Map model for different targets
to_upper_launch_model() {
  local m="${1:-}"
  local u="${m^^}"
  case "${u}" in
    O6) echo "O6" ;; 
    L6) echo "L6" ;;
    L7) echo "L7" ;;
    L10V6|L10V7) echo "L10" ;;
    L20) echo "L20" ;;
    L21) echo "L21" ;;
    L25) echo "L25" ;;
    *) echo "O6" ;;
  esac
}

to_lower_yaml_model() {
  local m="${1:-}"
  local u="${m^^}"
  case "${u}" in
    O6) echo "o6" ;; 
    L6) echo "l6" ;;
    L7) echo "l7" ;;
    L10V6) echo "l10v6" ;;
    L10V7) echo "l10v7" ;;
    L20) echo "l20" ;;
    L21) echo "l21" ;;
    L25) echo "l25" ;;
    *) echo "o6" ;;
  esac
}

# Determine values
LAUNCH_MODEL="$(to_upper_launch_model "${HAND_MODEL}")"
YAML_MODEL="$(to_lower_yaml_model "${HAND_MODEL}")"

# Python boolean literal for launch files
if [[ "${COLLECT_TACTILE}" == "true" ]]; then
  PY_BOOL="True"
else
  PY_BOOL="False"
fi

summary_changes=()

update_single_launch() {
  local file="$1"
  local hand_type="$2" # left|right
  local model="$3"     # uppercased mapped for launch
  local py_bool="$4"   # True|False (已弃用，linkerhand_cl不使用此参数)

  if [[ ! -f "${file}" ]]; then
    echo "Skip: ${file} not found" >&2
    return
  fi

  # linkerhand_cl 使用不同的参数格式
  # 更新 hand_type 和 hand_joint 参数
  sed -i -E "s/(hand_type[[:space:]]*=[[:space:]]*['\"])[^'\"]+(['\"])/\1${hand_type}\2/" "${file}"
  sed -i -E "s/(hand_joint[[:space:]]*=[[:space:]]*['\"])[^'\"]+(['\"])/\1${model}\2/" "${file}"

  summary_changes+=("${file}: hand_type=${hand_type}, hand_joint=${model} (linkerhand_cl)")
}

update_double_launch() {
  local file="$1"
  local model="$2"
  local py_bool="$3"  # 已弃用，linkerhand_cl不使用此参数
  if [[ ! -f "${file}" ]]; then
    echo "Skip: ${file} not found" >&2
    return
  fi
  # linkerhand_cl 使用不同的参数格式
  # 更新左右手的 hand_joint 参数
  sed -i -E "s/(left_hand_joint[[:space:]]*=[[:space:]]*['\"])[^'\"]+(['\"])/\1${model}\2/" "${file}"
  sed -i -E "s/(right_hand_joint[[:space:]]*=[[:space:]]*['\"])[^'\"]+(['\"])/\1${model}\2/" "${file}"
  summary_changes+=("${file}: left_hand_joint=${model}, right_hand_joint=${model} (linkerhand_cl)")
}

update_gui_launch_single_or_both() {
  local file="$1"
  local maybe_hand_type="$2" # empty means keep
  local model="$3"
  local py_bool="$4"
  if [[ ! -f "${file}" ]]; then
    echo "Skip: ${file} not found" >&2
    return
  fi
  if [[ -n "${maybe_hand_type}" ]]; then
    sed -i -E "s/('hand_type':[[:space:]]*)'[^']+'/\1'${maybe_hand_type}'/" "${file}"
  fi
  sed -i -E "s/('hand_joint':[[:space:]]*)\"[^\"]+\"/\1\"${model}\"/" "${file}"
  sed -i -E "s/('is_touch':[[:space:]]*)(True|False)/\1${py_bool}/" "${file}"

  if [[ -n "${maybe_hand_type}" ]]; then
    summary_changes+=("${file}: hand_type=${maybe_hand_type}, hand_joint=${model}, is_touch=${py_bool}")
  else
    summary_changes+=("${file}: hand_joint=${model}, is_touch=${py_bool}")
  fi
}

update_base_config() {
  local file="$1"
  local yaml_model="$2"
  local debug_r="$3" # true|false
  local debug_l="$4" # true|false
  # linker_telop_sdk 已移除，不再更新 base_config.yml
  # 此函数保留用于向后兼容，但不再执行实际操作
  if [[ ! -f "${file}" ]]; then
    echo "Skip: ${file} not found (linker_telop_sdk已移除)" >&2
    return
  fi
  echo "Note: linker_telop_sdk已移除，跳过base_config.yml更新" >&2
}

# Update setting.yaml for LEFT_HAND and RIGHT_HAND
update_setting_yaml() {
  local file="$1"
  local left_exists="$2"   # True|False
  local right_exists="$3"  # True|False
  local touch_bool="$4"    # True|False
  local joint_model="$5"   # O6/L6/L7/L10/L20/L21/L25

  if [[ ! -f "${file}" ]]; then
    echo "Skip: ${file} not found" >&2
    return
  fi

  # LEFT_HAND section: from LEFT_HAND: up to RIGHT_HAND:
  sed -i -E "/^[[:space:]]*LEFT_HAND:/,/^[[:space:]]*RIGHT_HAND:/ s/^([[:space:]]*EXISTS:[[:space:]]*)(True|False)/\1${left_exists}/" "${file}"
  sed -i -E "/^[[:space:]]*LEFT_HAND:/,/^[[:space:]]*RIGHT_HAND:/ s/^([[:space:]]*TOUCH:[[:space:]]*)(True|False)/\1${touch_bool}/" "${file}"
  sed -i -E "/^[[:space:]]*LEFT_HAND:/,/^[[:space:]]*RIGHT_HAND:/ s/^([[:space:]]*JOINT:[[:space:]]*)[A-Z0-9]+/\1${joint_model}/" "${file}"

  # RIGHT_HAND section: from RIGHT_HAND: to end of file
  sed -i -E "/^[[:space:]]*RIGHT_HAND:/,$ s/^([[:space:]]*EXISTS:[[:space:]]*)(True|False)/\1${right_exists}/" "${file}"
  sed -i -E "/^[[:space:]]*RIGHT_HAND:/,$ s/^([[:space:]]*TOUCH:[[:space:]]*)(True|False)/\1${touch_bool}/" "${file}"
  sed -i -E "/^[[:space:]]*RIGHT_HAND:/,$ s/^([[:space:]]*JOINT:[[:space:]]*)[A-Z0-9]+/\1${joint_model}/" "${file}"

  summary_changes+=("${file}: LEFT{EXISTS=${left_exists}, TOUCH=${touch_bool}, JOINT=${joint_model}} RIGHT{EXISTS=${right_exists}, TOUCH=${touch_bool}, JOINT=${joint_model}}")
}

# Apply according to arm type
case "${ARM_TYPE}" in
  piper)
    # Single hand; require HAND_SIDE left|right
    if [[ "${HAND_SIDE}" != "left" && "${HAND_SIDE}" != "right" ]]; then
      echo "For armType=piper, handSide must be 'left' or 'right'. Got: '${HAND_SIDE}'" >&2
      exit 2
    fi
    update_single_launch "${LH_LAUNCH_SINGLE}" "${HAND_SIDE}" "${LAUNCH_MODEL}" "${PY_BOOL}"
    update_gui_launch_single_or_both "${GUI_LAUNCH}" "${HAND_SIDE}" "${LAUNCH_MODEL}" "${PY_BOOL}"
    # linker_telop_sdk已移除，不再更新base_config.yml
    # setting.yaml更新（仅用于向后兼容）
    if [[ "${HAND_SIDE}" == "left" ]]; then
      update_setting_yaml "${SETTING_YAML}" "True" "False" "${PY_BOOL}" "${LAUNCH_MODEL}"
    else
      update_setting_yaml "${SETTING_YAML}" "False" "True" "${PY_BOOL}" "${LAUNCH_MODEL}"
    fi
    ;;
  linker)
    # Dual hands
    update_double_launch "${LH_LAUNCH_DOUBLE}" "${LAUNCH_MODEL}" "${PY_BOOL}"
    update_gui_launch_single_or_both "${GUI_LAUNCH}" "" "${LAUNCH_MODEL}" "${PY_BOOL}"
    # linker_telop_sdk已移除，不再更新base_config.yml
    # setting.yaml更新（仅用于向后兼容）
    update_setting_yaml "${SETTING_YAML}" "True" "True" "${PY_BOOL}" "${LAUNCH_MODEL}"
    ;;
  *)
    echo "Unknown armType: '${ARM_TYPE}'. Expected 'piper' or 'linker'." >&2
    exit 3
    ;;
esac

# Update task YAML files (double_linkerhand_grasp.yaml, linkerhand_piper_grasp.yaml, hardware_presets.yaml)
echo ""
echo "Updating task YAML configurations..."
# Activate virtual environment if available (for ruamel.yaml dependency)
if [ -f ~/.venv/data_collection/bin/activate ]; then
  source ~/.venv/data_collection/bin/activate
  if python3 scripts/update_task_configs.py 2>&1; then
    echo "✓ Task YAML configurations updated"
  else
    echo "⚠ Warning: Failed to update task YAML configurations"
  fi
  deactivate 2>/dev/null || true
else
  # Try without virtual environment (may fail if ruamel.yaml not installed)
  if python3 scripts/update_task_configs.py 2>&1; then
    echo "✓ Task YAML configurations updated"
  else
    echo "⚠ Warning: Failed to update task YAML configurations (may need ruamel.yaml: pip install ruamel.yaml)"
  fi
fi

echo ""
echo "Applied configuration from ${JSON_PATH}:"
for line in "${summary_changes[@]}"; do
  echo " - ${line}"
done


