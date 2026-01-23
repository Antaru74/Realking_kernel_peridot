#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
#
# merge_config.sh (CI / GKI safe version)
# - Never touches source tree
# - All temp files live in OUTPUT (O=)
# - Safe for Android GKI + GitHub Actions
#

set -e

usage() {
	echo "Usage: $0 [OPTIONS] BASE_CONFIG [FRAGMENTS...]"
	echo "  -h    display this help text"
	echo "  -m    only merge fragments, do not run make"
	echo "  -n    use allnoconfig instead of alldefconfig"
	echo "  -r    warn about redundant entries"
	echo "  -y    builtin (y) has precedence over modules (m)"
	echo "  -O    output directory (required for GKI/CI)"
	echo "  -s    strict mode (fail on redefinition)"
}

RUNMAKE=true
ALLTARGET=alldefconfig
WARNREDUN=false
BUILTIN=false
STRICT=false
OUTPUT=""
CONFIG_PREFIX=${CONFIG_-CONFIG_}

while true; do
	case "$1" in
		-h) usage; exit 0 ;;
		-m) RUNMAKE=false; shift ;;
		-n) ALLTARGET=allnoconfig; shift ;;
		-r) WARNREDUN=true; shift ;;
		-y) BUILTIN=true; shift ;;
		-s) STRICT=true; shift ;;
		-O)
			OUTPUT="$2"
			shift 2
			;;
		*) break ;;
	esac
done

if [ -z "$OUTPUT" ]; then
	echo "ERROR: -O <output_dir> is required (GKI safe mode)" >&2
	exit 1
fi

if [ "$#" -lt 1 ]; then
	usage
	exit 1
fi

BASE_CONFIG="$1"
shift
MERGE_LIST="$@"

if [ ! -r "$BASE_CONFIG" ]; then
	echo "Base config '$BASE_CONFIG' does not exist" >&2
	exit 1
fi

mkdir -p "$OUTPUT"

# Force everything into OUTPUT
export KCONFIG_CONFIG="$(readlink -m "$OUTPUT/.config")"

TMP_FILE="$(mktemp "$OUTPUT/.tmp.config.XXXXXXXXXX")"
MERGE_FILE="$(mktemp "$OUTPUT/.merge_tmp.config.XXXXXXXXXX")"

clean_up() {
	rm -f "$TMP_FILE" "$MERGE_FILE"
}
trap clean_up EXIT

SED_EXP1="s/^\(${CONFIG_PREFIX}[A-Za-z0-9_]*\)=.*/\1/p"
SED_EXP2="s/^# \(${CONFIG_PREFIX}[A-Za-z0-9_]*\) is not set$/\1/p"

echo "Using base config: $BASE_CONFIG"
cp "$BASE_CONFIG" "$TMP_FILE"

for FRAG in $MERGE_LIST; do
	echo "Merging fragment: $FRAG"

	if [ ! -r "$FRAG" ]; then
		echo "Fragment '$FRAG' does not exist" >&2
		exit 1
	fi

	cp "$FRAG" "$MERGE_FILE"
	CFG_LIST=$(sed -n -e "$SED_EXP1" -e "$SED_EXP2" "$MERGE_FILE")

	for CFG in $CFG_LIST; do
		grep -q -w "$CFG" "$TMP_FILE" || continue

		PREV_VAL=$(grep -w "$CFG" "$TMP_FILE")
		NEW_VAL=$(grep -w "$CFG" "$MERGE_FILE")

		if [ "$BUILTIN" = "true" ] &&
		   [ "${NEW_VAL#*=}" = "m" ] &&
		   [ "${PREV_VAL#*=}" = "y" ]; then
			echo "Keep builtin y over module m for $CFG"
			sed -i "/$CFG[ =]/d" "$MERGE_FILE"
			continue
		fi

		if [ "$PREV_VAL" != "$NEW_VAL" ]; then
			echo "Override $CFG:"
			echo "  old: $PREV_VAL"
			echo "  new: $NEW_VAL"
			if [ "$STRICT" = "true" ]; then
				exit 1
			fi
		elif [ "$WARNREDUN" = "true" ]; then
			echo "Redundant: $CFG"
		fi

		sed -i "/$CFG[ =]/d" "$TMP_FILE"
	done

	echo >> "$TMP_FILE"
	cat "$MERGE_FILE" >> "$TMP_FILE"
done

if [ "$RUNMAKE" = "false" ]; then
	cp "$TMP_FILE" "$KCONFIG_CONFIG"
	echo "#"
	echo "# merged configuration written to $KCONFIG_CONFIG"
	echo "#"
	exit 0
fi

echo "Running make $ALLTARGET (O=$OUTPUT)"
make O="$OUTPUT" KCONFIG_ALLCONFIG="$TMP_FILE" "$ALLTARGET"

# Verify results
for CFG in $(sed -n -e "$SED_EXP1" -e "$SED_EXP2" "$TMP_FILE"); do
	REQ=$(grep -w "$CFG" "$TMP_FILE")
	ACT=$(grep -w "$CFG" "$KCONFIG_CONFIG" || true)
	if [ "$REQ" != "$ACT" ]; then
		echo "WARNING: $CFG not applied"
		echo "  requested: $REQ"
		echo "  actual:    $ACT"
	fi
done
