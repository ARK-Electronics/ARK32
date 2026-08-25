#!/usr/bin/env bash
# Fetch the ARK 4IN1 bootloader from ARK-Electronics/ARK32-bootloader.
# Used by make when Bootloaders/*.bin is missing (not committed in this repo).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

OUT="${1:-Bootloaders/AM32_F051_BOOTLOADER_ARK4IN1_V18.bin}"
REPO="${BOOTLOADER_REPO:-https://github.com/ARK-Electronics/ARK32-bootloader.git}"
REF="${BOOTLOADER_REF:-cdce0a210f2fa5561ebc3a8c2b2c89943ff7d28b}"
SRC="${BOOTLOADER_SRC:-$ROOT/downloads/ARK32-bootloader}"
PRODUCT="${BOOTLOADER_PRODUCT:-AM32_F051_BOOTLOADER_ARK4IN1}"

if [[ -f "$OUT" ]]; then
	echo "bootloader image already present: $OUT"
	exit 0
fi

if ! command -v git >/dev/null 2>&1; then
	echo "error: git is required to fetch $REPO" >&2
	exit 1
fi

echo "Fetching ARK32-bootloader ($REF) into $SRC"
mkdir -p "$(dirname "$SRC")"
if [[ ! -d "$SRC/.git" ]]; then
	rm -rf "$SRC"
	git clone --depth 1 "$REPO" "$SRC"
fi
git -C "$SRC" fetch --depth 1 origin "$REF"
git -C "$SRC" checkout --detach FETCH_HEAD

MAKE_ARGS=()
if [[ -n "${ARM_SDK_PREFIX:-}" ]]; then
	# Prefix is often repo-relative (tools/linux/...); make -C would miss it.
	prefix="$ARM_SDK_PREFIX"
	if [[ "$prefix" != /* ]]; then
		prefix="$ROOT/$prefix"
	fi
	if [[ ! -x "${prefix}gcc" ]]; then
		echo "error: Arm gcc not found at ${prefix}gcc. Run: make arm_sdk_install" >&2
		exit 1
	fi
	MAKE_ARGS+=(ARM_SDK_PREFIX="$prefix")
fi

echo "Building $PRODUCT"
make -C "$SRC" "${MAKE_ARGS[@]}" -j"$(nproc 2>/dev/null || echo 4)" "$PRODUCT"

shopt -s nullglob
bins=("$SRC"/obj/"${PRODUCT}"_V*.bin)
if [[ ${#bins[@]} -ne 1 ]]; then
	echo "error: expected one $SRC/obj/${PRODUCT}_V*.bin, found ${#bins[@]}" >&2
	printf '  %s\n' "${bins[@]:-}" >&2
	exit 1
fi

mkdir -p "$(dirname "$OUT")"
cp -f "${bins[0]}" "$OUT"
echo "Wrote $OUT from ${bins[0]}"
