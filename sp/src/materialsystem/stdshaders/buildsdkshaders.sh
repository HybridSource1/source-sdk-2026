#!/usr/bin/env bash
# Linux version of buildsdkshaders.bat. Valve's Perl steps run natively; only
# shadercompile.exe runs under Wine.
#
# Usage: ./buildsdkshaders.sh <game folder with gameinfo.txt> <folder with shadercompile.exe>
#
# The second folder is the bin folder of a Windows install of Source SDK Base 2013
# Singleplayer: shadercompile.exe, shadercompile_dll.dll, tier0.dll, vstdlib.dll.
#
# Needs: perl with String::CRC32 (Debian/Ubuntu: libstring-crc32-perl),
#        32-bit Wine (Debian/Ubuntu: wine32:i386).
set -euo pipefail

if [ $# -lt 2 ]; then
	sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
	exit 2
fi

HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../.." && pwd)
GAMEDIR=$(cd "$1" && pwd)
SDKBIN=$(cd "$2" && pwd)

die() { echo "error: $*" >&2; exit 1; }
[ -f "$GAMEDIR/gameinfo.txt" ] || die "$GAMEDIR has no gameinfo.txt"
for f in shadercompile.exe shadercompile_dll.dll tier0.dll vstdlib.dll; do
	[ -f "$SDKBIN/$f" ] || die "$SDKBIN is missing $f"
done
command -v perl >/dev/null || die "perl is not installed"
perl -MString::CRC32 -e1 2>/dev/null || die "Perl module String::CRC32 is missing (libstring-crc32-perl)"
command -v wine >/dev/null || die "wine is not installed"
command -v winepath >/dev/null || die "winepath (part of wine) is not installed"

export WINEDEBUG=${WINEDEBUG:--all}
winpath() { winepath -w "$1" 2>/dev/null; }

# Valve's scripts write some paths with backslashes, which on Linux become part of
# the file name. Move such files to where they were meant to go.
fix_backslash_names() {
	local f
	while IFS= read -r -d '' f; do
		local name=${f#./}
		local dest=${name//\\//}
		mkdir -p "$(dirname "$dest")"
		mv -f -- "$name" "$dest"
	done < <(find . -maxdepth 1 -name '*\\*' -print0)
}

# Runs the recipes of the nmake makefile that updateshaders.pl generates. Every
# rule runs, which is what a clean build does anyway.
run_makefile() {
	local line
	while IFS= read -r line; do
		line=${line%$'\r'}
		[[ $line == $'\t'* ]] || continue
		line=${line#$'\t'}
		line=${line//\\//}
		bash -c "$line"
	done < "$1"
}

build_list() {
	local list=$1 force30=$2
	echo "==================== $list ===================="
	cd "$HERE"
	mkdir -p shaders/fxc shaders/vsh shaders/psh fxctmp9 fxctmp9_tmp vshtmp9 vshtmp9_tmp
	rm -f filelist.txt filestocopy.txt filelistgen.txt inclist.txt vcslist.txt uniquefilestocopy.txt

	perl "$SRC/devtools/bin/updateshaders.pl" -source "$SRC" "$list"
	echo "Building inc files and the work list..."
	run_makefile "makefile.$list"
	fix_backslash_names

	# What copyshaderincfiles.pl does: publish headers that changed.
	if [ -f inclist.txt ]; then
		local inc dst existing
		while IFS= read -r inc; do
			inc=${inc%$'\r'}; inc=${inc//\\//}
			[ -n "$inc" ] && [ -f "$inc" ] || continue
			dst=${inc/_tmp/}
			# Windows ignores case, and some shader names differ in case only.
			existing=$(find "$(dirname "$dst")" -maxdepth 1 -iname "$(basename "$dst")" -print -quit)
			[ -n "$existing" ] && dst=$existing
			if ! cmp -s "$inc" "$dst"; then
				echo "Updated header: $dst"
				cp -f "$inc" "$dst"
			fi
		done < inclist.txt
	fi

	[ -f filelistgen.txt ] || { echo "Nothing to compile."; return; }

	# shadercompile copies these to its workers; with -nompi it only checks them.
	{
		[ -f filestocopy.txt ] && tr -d '\r' < filestocopy.txt
		winpath "$SRC/devtools/bin/d3dx9_33.dll"
		winpath "$SRC/dx9sdk/utilities/dx_proxy.dll"
		for f in shadercompile.exe shadercompile_dll.dll vstdlib.dll tier0.dll; do winpath "$SDKBIN/$f"; done
	} | awk 'NF && !seen[tolower($0)]++' | sed 's/$/\r/' > uniquefilestocopy.txt
	cp -f filelistgen.txt filelist.txt

	echo "Running shadercompile under Wine..."
	(
		cd "$SDKBIN"
		if [ "$force30" = 1 ]; then export DIRECTX_FORCE_MODEL=30; fi
		wine shadercompile.exe -nompi -nop4 -game "$(winpath "$GAMEDIR")" -shaderpath "$(winpath "$HERE")" -allowdebug
	)

	mkdir -p "$GAMEDIR/shaders"
	cp -rf shaders/. "$GAMEDIR/shaders/"
}

build_list stdshader_dx9_20b 0
build_list stdshader_dx9_30 1
echo "Done. Shaders are in $GAMEDIR/shaders"
