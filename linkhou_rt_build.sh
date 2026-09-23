#!/bin/bash
# linkhou_rt_build.sh
# Control PREEMPT_RT in kernel defconfig + apply/revert rt-patches
# according to MY_IS_BUILD_RT_KERNEL from my_bsp.conf.
#
# Usage:
#   ./linkhou_rt_build.sh              # read MY_IS_BUILD_RT_KERNEL
#   ./linkhou_rt_build.sh enable|disable|status
#
set -euo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "${0}")")"
source "${SCRIPT_DIR}/kernel_src_build_env.sh"

BSP_CONF="$(realpath "${SCRIPT_DIR}/../my_bsp.conf")"
if [ -f "${BSP_CONF}" ]; then
	# shellcheck source=/dev/null
	source "${BSP_CONF}"
fi

CFG_DIR="${SCRIPT_DIR}/kernel/${KERNEL_SRC_DIR}/arch/arm64/configs"
KERNEL_DIR="${SCRIPT_DIR}/kernel/${KERNEL_SRC_DIR}"
CFG_TOOL="${KERNEL_DIR}/scripts/config"
STATE_FILE="${CFG_DIR}/.linkhou_rt_state"
PATCH_MARK="${CFG_DIR}/.linkhou_rt_patches_applied"
DEFCONFIG="${CFG_DIR}/defconfig"
PROD_DEFCONFIG="${CFG_DIR}/tegra_prod_defconfig"
any_failure=0

want_rt_from_env() {
	if [ "${MY_IS_BUILD_RT_KERNEL:-0}" = "1" ]; then
		echo 1
	else
		echo 0
	fi
}

get_state() {
	if [ -f "${STATE_FILE}" ]; then
		cat "${STATE_FILE}"
	else
		echo "unknown"
	fi
}

set_state() {
	echo "${1}" > "${STATE_FILE}"
}

has_preempt_rt_config() {
	grep -q "^CONFIG_PREEMPT_RT=y" "${1}" 2>/dev/null
}

apply_rt_configs_to_file() {
	local f="${1}"
	"${CFG_TOOL}" --file "${f}" \
		--enable PREEMPT_RT --disable DEBUG_PREEMPT \
		--disable KVM \
		--enable EMBEDDED \
		--enable EXPERT \
		--enable NAMESPACES \
		--enable OSNOISE_TRACER \
		--enable TIMERLAT_TRACER \
		--disable CPU_IDLE_TEGRA18X \
		--disable CPU_FREQ_GOV_INTERACTIVE \
		--disable CPU_FREQ_TIMES \
		--disable FAIR_GROUP_SCHED || any_failure=1
}

apply_nonrt_configs_to_file() {
	local f="${1}"
	# Drop RT preempt choice; use standard PREEMPT. Reverse RT-oriented toggles.
	"${CFG_TOOL}" --file "${f}" \
		--disable PREEMPT_RT --enable PREEMPT \
		--enable DEBUG_PREEMPT \
		--enable KVM \
		--enable EMBEDDED \
		--enable EXPERT \
		--enable NAMESPACES \
		--disable OSNOISE_TRACER \
		--disable TIMERLAT_TRACER \
		--enable CPU_IDLE_TEGRA18X \
		--enable CPU_FREQ_GOV_INTERACTIVE \
		--enable CPU_FREQ_TIMES \
		--enable FAIR_GROUP_SCHED || any_failure=1
}

refresh_defconfig_links() {
	# Keep tegra_defconfig as symlink/copy of defconfig (same as generic flow)
	rm -f "${CFG_DIR}/tegra_defconfig"
	ln -s "defconfig" "${CFG_DIR}/tegra_defconfig"
}

apply_rt_patches() {
	if [ -f "${PATCH_MARK}" ]; then
		echo "[linkhou_rt] RT patches already marked applied, skip"
		return 0
	fi
	if [ ! -d "${KERNEL_DIR}/rt-patches" ]; then
		echo "[linkhou_rt] No rt-patches directory, skip patch apply"
		return 0
	fi
	local p
	local file_list
	file_list="$(find "${KERNEL_DIR}/rt-patches" -name '*.patch' -type f | sort)"
	pushd "${SCRIPT_DIR}" >/dev/null
	for p in ${file_list}; do
		echo "[linkhou_rt] apply $(basename "${p}")"
		if ! patch -s -d .. -p1 < "${p}"; then
			echo "[linkhou_rt] failed patching ${p}"
			any_failure=1
			popd >/dev/null
			return 1
		fi
	done
	popd >/dev/null
	touch "${PATCH_MARK}"
	echo "[linkhou_rt] RT patches applied"
}

revert_rt_patches() {
	if [ ! -f "${PATCH_MARK}" ]; then
		echo "[linkhou_rt] RT patches not marked applied, skip revert"
		return 0
	fi
	if [ ! -d "${KERNEL_DIR}/rt-patches" ]; then
		rm -f "${PATCH_MARK}"
		return 0
	fi
	local p
	local file_list
	file_list="$(find "${KERNEL_DIR}/rt-patches" -name '*.patch' -type f | sort -r)"
	pushd "${SCRIPT_DIR}" >/dev/null
	for p in ${file_list}; do
		echo "[linkhou_rt] revert $(basename "${p}")"
		# Best-effort: some trees already contain RT patches baked in
		patch -s -R -d .. -p1 < "${p}" || echo "[linkhou_rt] warn: revert may already be done for ${p}"
	done
	popd >/dev/null
	rm -f "${PATCH_MARK}"
	echo "[linkhou_rt] RT patches reverted (or already clean)"
}

enable_rt() {
	echo "[linkhou_rt] Enable RT kernel"
	if [ "$(get_state)" = "rt" ] && has_preempt_rt_config "${DEFCONFIG}"; then
		echo "[linkhou_rt] already in RT state"
		return 0
	fi

	# Backup originals once for reference / generic_rt compatibility
	if [ ! -f "${CFG_DIR}/.orig.defconfig" ]; then
		cp -f "${DEFCONFIG}" "${CFG_DIR}/.orig.defconfig"
		cp -f "${PROD_DEFCONFIG}" "${CFG_DIR}/.orig.tegra_prod_defconfig"
	fi

	apply_rt_patches || return 1
	apply_rt_configs_to_file "${DEFCONFIG}"
	apply_rt_configs_to_file "${PROD_DEFCONFIG}"
	cp -f "${DEFCONFIG}" "${CFG_DIR}/.updated.defconfig"
	cp -f "${PROD_DEFCONFIG}" "${CFG_DIR}/.updated.tegra_prod_defconfig"
	refresh_defconfig_links
	set_state "rt"
	echo "[linkhou_rt] PREEMPT RT enabled successfully"
}

disable_rt() {
	echo "[linkhou_rt] Disable RT kernel"
	if [ "$(get_state)" = "nonrt" ] && ! has_preempt_rt_config "${DEFCONFIG}"; then
		echo "[linkhou_rt] already in non-RT state"
		return 0
	fi

	revert_rt_patches

	# Always force non-RT configs (do not rely on .orig which may already contain PREEMPT_RT)
	apply_nonrt_configs_to_file "${DEFCONFIG}"
	apply_nonrt_configs_to_file "${PROD_DEFCONFIG}"
	refresh_defconfig_links

	rm -f "${CFG_DIR}/.orig.defconfig" \
		"${CFG_DIR}/.updated.defconfig" \
		"${CFG_DIR}/.orig.tegra_prod_defconfig" \
		"${CFG_DIR}/.updated.tegra_prod_defconfig"

	set_state "nonrt"
	if has_preempt_rt_config "${DEFCONFIG}"; then
		echo "[linkhou_rt] ERROR: CONFIG_PREEMPT_RT still set in defconfig"
		any_failure=1
		return 1
	fi
	echo "[linkhou_rt] PREEMPT RT disabled successfully"
}

show_status() {
	echo "[linkhou_rt] MY_IS_BUILD_RT_KERNEL=${MY_IS_BUILD_RT_KERNEL:-unset}"
	echo "[linkhou_rt] state=$(get_state)"
	echo "[linkhou_rt] patches_mark=$([ -f "${PATCH_MARK}" ] && echo yes || echo no)"
	if has_preempt_rt_config "${DEFCONFIG}"; then
		echo "[linkhou_rt] defconfig: CONFIG_PREEMPT_RT=y"
	else
		echo "[linkhou_rt] defconfig: CONFIG_PREEMPT_RT not set"
	fi
	grep -E '^CONFIG_PREEMPT' "${DEFCONFIG}" || true
}

# ---- main ----
cmd="${1:-auto}"
case "${cmd}" in
	enable)
		enable_rt
		show_status
		;;
	disable)
		disable_rt
		show_status
		;;
	status)
		show_status
		;;
	auto)
		if [ "$(want_rt_from_env)" = "1" ]; then
			enable_rt
		else
			disable_rt
		fi
		show_status
		;;
	*)
		echo "Usage: $0 [auto|enable|disable|status]"
		exit 1
		;;
esac

exit "${any_failure}"
