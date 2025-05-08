#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `anveshika_description` package

TMP_URDF_PATH=$(mktemp /tmp/anveshika_XXXXXX.urdf)

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
PKG_PATH=$(ros2 pkg prefix anveshika_description)
xacro "${1}" -o "${TMP_URDF_PATH}" &&
SDF_XML=$(gz sdf -p "${TMP_URDF_PATH}" | sed "s|model://anveshika_description|file://${PKG_PATH}/share/anveshika_description|g")

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null

# Return SDF as XML string
echo "${SDF_XML}"