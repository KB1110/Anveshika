#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `anveshika_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/anveshika.urdf.xacro"
URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/anveshika.urdf"

# Remove old URDF file
rm "${URDF_PATH}" 2>/dev/null

# Process xacro into URDF
xacro "${XACRO_PATH}" -o "${URDF_PATH}" &&
echo "Created new ${URDF_PATH}"