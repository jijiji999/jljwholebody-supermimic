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

resolve_cannelloni_bin() {
  if [[ -n "${CANNELLONI_BIN:-}" && -x "${CANNELLONI_BIN}" ]]; then
    echo "${CANNELLONI_BIN}"
    return 0
  fi

  if command -v cannelloni >/dev/null 2>&1; then
    command -v cannelloni
    return 0
  fi

  local workspace_root
  workspace_root="${WORKSPACE_ROOT:-$(find_workspace_root || true)}"
  if [[ -n "${workspace_root}" ]]; then
    local local_bin="${workspace_root}/third_party/cannelloni/build/cannelloni"
    if [[ -x "${local_bin}" ]]; then
      echo "${local_bin}"
      return 0
    fi
  fi

  return 1
}

stop_existing_bridge() {
  local pid_file="$1"
  if [[ ! -f "${pid_file}" ]]; then
    return 0
  fi

  local existing_pid
  existing_pid="$(cat "${pid_file}")"
  if [[ "${existing_pid}" =~ ^[0-9]+$ ]] && kill -0 "${existing_pid}" 2>/dev/null; then
    echo "Stopping existing bridge process ${existing_pid} from ${pid_file}"
    kill "${existing_pid}"

    for _ in $(seq 1 20); do
      if ! kill -0 "${existing_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
  fi

  rm -f "${pid_file}"
}

REMOTE_IP="${1:-192.168.1.157}"
BASE_PORT="${2:-14000}"
BUFFER_TIMEOUT_US="${3:-50}"
INTERFACE_PREFIX="${INTERFACE_PREFIX:-vcan}"
INTERFACE_COUNT="${INTERFACE_COUNT:-4}"
ETHERNET_INTERFACE="${ETHERNET_INTERFACE:-enp8s0}"

if ! ip link show dev "${ETHERNET_INTERFACE}" >/dev/null 2>&1; then
  echo "Physical Ethernet interface ${ETHERNET_INTERFACE} does not exist." >&2
  exit 1
fi

if ! grep -qx 1 "/sys/class/net/${ETHERNET_INTERFACE}/carrier" 2>/dev/null; then
  echo "Physical Ethernet interface ${ETHERNET_INTERFACE} has no carrier; LinkX is disconnected." >&2
  exit 1
fi

if ! ping -c 1 -W 1 "${REMOTE_IP}" >/dev/null 2>&1; then
  echo "LinkX ${REMOTE_IP} is not reachable through ${ETHERNET_INTERFACE}." >&2
  exit 1
fi

if ! [[ "${BASE_PORT}" =~ ^[0-9]+$ ]]; then
  echo "BASE_PORT must be an integer." >&2
  exit 1
fi

if ! [[ "${BUFFER_TIMEOUT_US}" =~ ^[0-9]+$ ]]; then
  echo "BUFFER_TIMEOUT_US must be an integer in microseconds." >&2
  exit 1
fi

if ! [[ "${INTERFACE_COUNT}" =~ ^[0-9]+$ ]] || [[ "${INTERFACE_COUNT}" -le 0 ]]; then
  echo "INTERFACE_COUNT must be a positive integer." >&2
  exit 1
fi

if ! CANNELLONI_BIN="$(resolve_cannelloni_bin)"; then
  echo "Could not find cannelloni binary." >&2
  echo "Build it with: bash src/jljwholebody_bringup/scripts/build_cannelloni.sh" >&2
  exit 1
fi

echo "========================================="
echo " Initializing ${INTERFACE_COUNT} SocketCAN virtual buses"
echo "========================================="
echo "Using cannelloni binary: ${CANNELLONI_BIN}"

sudo modprobe vcan

for ((i = 0; i < INTERFACE_COUNT; ++i)); do
  iface="${INTERFACE_PREFIX}${i}"
  if ! ip link show "${iface}" >/dev/null 2>&1; then
    sudo ip link add dev "${iface}" type vcan
    echo "Created ${iface}"
  fi

  sudo ip link set up "${iface}"
  echo "Brought up ${iface}"
done

echo
echo "========================================="
echo " Creating Cannelloni UDP tunnels         "
echo " Remote IP: ${REMOTE_IP}"
echo " Base port: ${BASE_PORT}"
echo " Buffer timeout: ${BUFFER_TIMEOUT_US} us"
echo "========================================="

for ((i = 0; i < INTERFACE_COUNT; ++i)); do
  iface="${INTERFACE_PREFIX}${i}"
  port=$((BASE_PORT + i))
  pid_file="/tmp/cannelloni_${iface}.pid"
  log_file="/tmp/cannelloni_${iface}.log"

  stop_existing_bridge "${pid_file}"

  "${CANNELLONI_BIN}" \
    -I "${iface}" \
    -R "${REMOTE_IP}" \
    -r "${port}" \
    -l "${port}" \
    -t "${BUFFER_TIMEOUT_US}" \
    -f \
    -P "${pid_file}" >"${log_file}" 2>&1

  echo "Started tunnel for ${iface} on UDP ${port} (pid file: ${pid_file}, log: ${log_file})"
done

echo
echo "All CAN FD bridge processes are running in background."
echo "Stop them with: pkill cannelloni"
