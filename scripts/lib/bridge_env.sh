#!/usr/bin/env bash

bridge_env_is_true() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

bridge_env_load() {
  local root_dir="$1"
  local env_file="${2:-${root_dir}/config/bridge.env}"

  if [[ ! -f "${env_file}" ]]; then
    echo "Missing ${env_file}"
    echo "Copy config/bridge.env.example to config/bridge.env and edit it for your network."
    return 1
  fi

  # shellcheck source=/dev/null
  source "${env_file}"
  bridge_env_resolve_pi_host
}

bridge_env_resolve_pi_host() {
  local configured_host="${PI_HOST:-auto}"
  local known_hosts="${PI_KNOWN_HOSTS:-${PI_PREVIOUS_HOSTS:-}}"
  local resolved_host=""

  if [[ -n "${configured_host}" && "${configured_host}" != "auto" ]]; then
    if bridge_env_should_refresh_pi_host "${configured_host}"; then
      resolved_host="$(bridge_env_discover_pi_by_known_ssh_key "${configured_host}")" || true
      if [[ -n "${resolved_host}" ]]; then
        PI_HOST="${resolved_host}"
        export PI_HOST
        echo "Resolved stale Pi host ${configured_host} -> ${PI_HOST} by matching the SSH host key."
        return 0
      fi
      echo "Warning: Pi host ${configured_host} did not answer on SSH and auto-refresh did not find a replacement." >&2
    fi

    PI_HOST="${configured_host}"
    export PI_HOST
    return 0
  fi

  if [[ -n "${known_hosts}" ]]; then
    resolved_host="$(bridge_env_discover_pi_by_known_ssh_key "${known_hosts}")" || true
  fi

  if [[ -z "${resolved_host}" ]]; then
    resolved_host="$(bridge_env_resolve_pi_by_name)" || true
  fi

  if [[ -n "${resolved_host}" ]]; then
    PI_HOST="${resolved_host}"
    export PI_HOST
    echo "Resolved Pi host automatically: ${PI_HOST}"
    return 0
  fi

  cat >&2 <<EOF
PI_HOST is set to auto, but the Pi could not be discovered.
Set PI_MDNS_HOSTS to the Pi's .local hostname, set PI_KNOWN_HOSTS to a previous Pi address, or set PI_HOST to the current IP.
EOF
  return 1
}

bridge_env_should_refresh_pi_host() {
  local configured_host="$1"

  if ! bridge_env_is_true "${PI_HOST_AUTO_REFRESH:-true}"; then
    return 1
  fi

  if bridge_env_known_host_has_key "${configured_host}"; then
    ! bridge_env_host_matches_known_ssh_key "${configured_host}" "${configured_host}"
    return $?
  fi

  ! bridge_env_tcp_open "${configured_host}" "${PI_SSH_PORT:-22}" "${PI_HOST_CONNECT_TIMEOUT:-1}"
}

bridge_env_resolve_pi_by_name() {
  local raw_hosts="${PI_MDNS_HOSTS:-}"
  local host=""
  local resolved=""

  if [[ -n "${PI_MDNS_HOST:-}" ]]; then
    raw_hosts="${raw_hosts} ${PI_MDNS_HOST}"
  fi
  if [[ -n "${PI_HOSTNAME:-}" ]]; then
    raw_hosts="${raw_hosts} ${PI_HOSTNAME}"
    if [[ "${PI_HOSTNAME}" != *.* ]]; then
      raw_hosts="${raw_hosts} ${PI_HOSTNAME}.local"
    fi
  fi
  raw_hosts="${raw_hosts:-raspberrypi.local pi.local raspi.local}"
  raw_hosts="${raw_hosts//,/ }"

  for host in ${raw_hosts}; do
    resolved="$(bridge_env_resolve_ipv4 "${host}")" || true
    if [[ -n "${resolved}" ]]; then
      echo "${resolved}"
      return 0
    fi
  done

  return 1
}

bridge_env_resolve_ipv4() {
  local host="$1"
  local resolved=""

  if [[ "${host}" =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
    echo "${host}"
    return 0
  fi

  if command -v getent >/dev/null 2>&1; then
    resolved="$(getent ahostsv4 "${host}" 2>/dev/null | awk '$1 ~ /^[0-9.]+$/ { print $1; exit }')" || true
    if [[ -n "${resolved}" ]]; then
      echo "${resolved}"
      return 0
    fi
  fi

  if command -v avahi-resolve-host-name >/dev/null 2>&1; then
    resolved="$(avahi-resolve-host-name -4 "${host}" 2>/dev/null | awk '$2 ~ /^[0-9.]+$/ { print $2; exit }')" || true
    if [[ -n "${resolved}" ]]; then
      echo "${resolved}"
      return 0
    fi
  fi

  if command -v host >/dev/null 2>&1; then
    resolved="$(host -t A "${host}" 2>/dev/null | awk '/has address/ { print $4; exit }')" || true
    if [[ -n "${resolved}" ]]; then
      echo "${resolved}"
      return 0
    fi
  fi

  return 1
}

bridge_env_tcp_open() {
  local host="$1"
  local port="$2"
  local timeout_s="$3"

  if command -v nc >/dev/null 2>&1; then
    nc -z -w "${timeout_s}" "${host}" "${port}" >/dev/null 2>&1
    return $?
  fi

  if command -v timeout >/dev/null 2>&1; then
    timeout "${timeout_s}" bash -c ":</dev/tcp/${host}/${port}" >/dev/null 2>&1
    return $?
  fi

  return 1
}

bridge_env_discover_pi_by_known_ssh_key() {
  local raw_known_hosts="$1"
  local known_keys_file=""
  local scan_hosts_file=""
  local scan_output_file=""
  local discovered=""

  raw_known_hosts="${raw_known_hosts//,/ }"
  if [[ -z "${raw_known_hosts// }" ]] || ! command -v ssh-keygen >/dev/null 2>&1 || ! command -v ssh-keyscan >/dev/null 2>&1; then
    return 1
  fi

  known_keys_file="$(mktemp)"
  scan_hosts_file="$(mktemp)"
  scan_output_file="$(mktemp)"

  bridge_env_collect_known_ssh_keys "${raw_known_hosts}" >"${known_keys_file}"
  if [[ ! -s "${known_keys_file}" ]]; then
    rm -f "${known_keys_file}" "${scan_hosts_file}" "${scan_output_file}"
    return 1
  fi

  bridge_env_scan_hosts >"${scan_hosts_file}"
  if [[ ! -s "${scan_hosts_file}" ]]; then
    rm -f "${known_keys_file}" "${scan_hosts_file}" "${scan_output_file}"
    return 1
  fi

  ssh-keyscan \
    -T "${PI_HOST_SCAN_TIMEOUT:-1}" \
    -p "${PI_SSH_PORT:-22}" \
    -t rsa,ecdsa,ed25519 \
    -f "${scan_hosts_file}" \
    >"${scan_output_file}" 2>/dev/null || true

  discovered="$(bridge_env_match_scanned_ssh_key "${known_keys_file}" "${scan_output_file}")" || true
  rm -f "${known_keys_file}" "${scan_hosts_file}" "${scan_output_file}"

  if [[ -n "${discovered}" ]]; then
    echo "${discovered}"
    return 0
  fi

  return 1
}

bridge_env_known_host_has_key() {
  local known_host="$1"

  [[ -n "$(bridge_env_collect_known_ssh_keys "${known_host}")" ]]
}

bridge_env_host_matches_known_ssh_key() {
  local known_host="$1"
  local candidate_host="$2"
  local known_keys_file=""
  local scan_output_file=""
  local matched=""

  known_keys_file="$(mktemp)"
  scan_output_file="$(mktemp)"

  bridge_env_collect_known_ssh_keys "${known_host}" >"${known_keys_file}"
  if [[ -s "${known_keys_file}" ]]; then
    ssh-keyscan \
      -T "${PI_HOST_SCAN_TIMEOUT:-1}" \
      -p "${PI_SSH_PORT:-22}" \
      -t rsa,ecdsa,ed25519 \
      "${candidate_host}" \
      >"${scan_output_file}" 2>/dev/null || true
    matched="$(bridge_env_match_scanned_ssh_key "${known_keys_file}" "${scan_output_file}")" || true
  fi

  rm -f "${known_keys_file}" "${scan_output_file}"
  [[ -n "${matched}" ]]
}

bridge_env_collect_known_ssh_keys() {
  local known_hosts="$1"
  local host=""

  for host in ${known_hosts}; do
    ssh-keygen -F "${host}" 2>/dev/null |
      awk '
        NF >= 3 && $1 !~ /^#/ {
          if ($1 ~ /^@/ && NF >= 4) {
            print $3 " " $4
          } else {
            print $2 " " $3
          }
        }
      '
  done | sort -u
}

bridge_env_scan_hosts() {
  local raw_cidrs="${PI_HOST_SCAN_CIDRS:-}"
  local cidr=""

  if [[ -z "${raw_cidrs}" ]] && command -v ip >/dev/null 2>&1; then
    raw_cidrs="$(ip -o -4 addr show scope global | awk '{ print $4 }')"
  fi
  raw_cidrs="${raw_cidrs//,/ }"

  for cidr in ${raw_cidrs}; do
    bridge_env_expand_scan_cidr "${cidr}"
  done | sort -u
}

bridge_env_expand_scan_cidr() {
  local cidr="$1"
  local prefix=""
  local i=0

  if [[ "${cidr}" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)\.[0-9]+/24$ ]]; then
    prefix="${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[3]}"
    for i in {1..254}; do
      echo "${prefix}.${i}"
    done
  fi
}

bridge_env_match_scanned_ssh_key() {
  local known_keys_file="$1"
  local scan_output_file="$2"

  awk '
    FNR == NR {
      known[$1 " " $2] = 1
      next
    }
    NF >= 3 {
      key = $2 " " $3
      if (key in known) {
        host = $1
        sub(/^\[/, "", host)
        sub(/\]:[0-9]+$/, "", host)
        print host
        exit 0
      }
    }
  ' "${known_keys_file}" "${scan_output_file}"
}
