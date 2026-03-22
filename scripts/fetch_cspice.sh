#!/usr/bin/env bash
# Download and install CSPICE N0067 (Jan 2022) into the astro submodule.
# Required after a fresh clone — the libraries/ directory is gitignored in astro.
#
# Steps follow the astro README:
#   https://github.com/LarsFlaeten/astro#spice-toolkit

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$SCRIPT_DIR/../libs/astro/libraries/cspice"
NAIF_URL="https://naif.jpl.nasa.gov/pub/naif/toolkit//C/PC_Linux_GCC_64bit/packages/cspice.tar.Z"
TARBALL="/tmp/cspice_n0067.tar.Z"

if [[ -f "$DEST/libcspice.a" ]]; then
    echo "CSPICE already present at $DEST — nothing to do."
    exit 0
fi

echo "Downloading CSPICE N0067 from NAIF (~3 MB compressed)..."
curl -fsSL "$NAIF_URL" -o "$TARBALL"

echo "Extracting..."
TMPDIR=$(mktemp -d)
uncompress -c "$TARBALL" | tar -x -C "$TMPDIR"

echo "Installing headers and library to $DEST ..."
mkdir -p "$DEST"
cp "$TMPDIR/cspice/include/"*.h "$DEST/"
cp "$TMPDIR/cspice/lib/cspice.a" "$DEST/libcspice.a"

rm -rf "$TMPDIR" "$TARBALL"
echo "Done. CSPICE N0067 installed."
