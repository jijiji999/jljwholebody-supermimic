#!/bin/bash

set -euo pipefail

find_workspace_root() {
  local current_dir
  current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

  while [[ "${current_dir}" != "/" ]]; do
    if [[ -f "${current_dir}/third_party/cannelloni/CMakeLists.txt" ]]; then
      echo "${current_dir}"
      return 0
    fi
    current_dir="$(dirname "${current_dir}")"
  done

  return 1
}

if [[ -z "${WORKSPACE_ROOT:-}" ]]; then
  WORKSPACE_ROOT="$(find_workspace_root || true)"
  if [[ -z "${WORKSPACE_ROOT}" ]]; then
    # On a first build the third_party checkout does not exist yet, so the
    # marker-based search above cannot identify the workspace.
    WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
  fi
fi
if [[ -z "${WORKSPACE_ROOT}" ]]; then
  echo "Could not find workspace root containing third_party/cannelloni." >&2
  exit 1
fi

CANNELLONI_REPO_URL="${CANNELLONI_REPO_URL:-https://github.com/mguentner/cannelloni.git}"
CANNELLONI_SOURCE_DIR="${WORKSPACE_ROOT}/third_party/cannelloni"
CANNELLONI_BUILD_DIR="${CANNELLONI_SOURCE_DIR}/build"

if [[ ! -d "${CANNELLONI_SOURCE_DIR}" ]]; then
  mkdir -p "${WORKSPACE_ROOT}/third_party"
  git clone "${CANNELLONI_REPO_URL}" "${CANNELLONI_SOURCE_DIR}"
fi

cmake \
  -S "${CANNELLONI_SOURCE_DIR}" \
  -B "${CANNELLONI_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DSCTP_SUPPORT=OFF

cmake --build "${CANNELLONI_BUILD_DIR}" -j"$(nproc)"

echo "Built cannelloni at: ${CANNELLONI_BUILD_DIR}/cannelloni"
