#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `anveshika_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/anveshika.urdf.xacro"
SDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/model.sdf"

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
"${SCRIPT_DIR}/xacro2sdf_direct.bash" "${XACRO_PATH}" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"