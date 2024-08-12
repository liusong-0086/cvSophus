#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")"; pwd -P)"

export PROJECT_ROOT="${SCRIPT_DIR}"
THIRD_PARTY_DIR="${PROJECT_ROOT}/thirdparty"

if [[ -z "${BUILD_TYPE}" ]]; then
  export BUILD_TYPE="Release"
fi

# export PATH="${THIRD_PARTY_DIR}/genicam/bin":$PATH
# export PATH="${THIRD_PARTY_DIR}/opencv/build/x64/vc16/bin":$PATH
# export PATH="${THIRD_PARTY_DIR}/Diana/bin":$PATH
export PATH="${PROJECT_ROOT}/robot2.12/bin":$PATH


TARGET=$1
shift

"${SCRIPT_DIR}/scripts/build.sh" "$TARGET"
# "${SCRIPT_DIR}/scripts/install.sh" "$TARGET"

cd "$PROJECT_ROOT/install/bin"

"./$TARGET.exe" "$@"