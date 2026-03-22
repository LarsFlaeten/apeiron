#!/usr/bin/env bash
# Download SPICE kernels for Apeiron.
# SPK binaries are gitignored due to size — run this script after cloning.
# All kernels sourced from NAIF: https://naif.jpl.nasa.gov/pub/naif/generic_kernels/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="$SCRIPT_DIR/../data/kernels"

NAIF="https://naif.jpl.nasa.gov/pub/naif/generic_kernels"

download() {
    local url="$1"
    local dest="$2"
    if [[ -f "$dest" ]]; then
        echo "  already exists: $dest"
    else
        echo "  downloading: $(basename "$dest")"
        curl -fsSL "$url" -o "$dest"
    fi
}

echo "Fetching SPICE kernels into $DATA_DIR ..."

# Leap second kernel
download "$NAIF/lsk/naif0012.tls" "$DATA_DIR/lsk/naif0012.tls"

# Planetary constants (orientation, radii, pole positions)
download "$NAIF/pck/pck00011.tpc"  "$DATA_DIR/pck/pck00011.tpc"

# GM values matching de440
download "$NAIF/pck/gm_de440.tpc"  "$DATA_DIR/pck/gm_de440.tpc"

# Planetary ephemeris — de440s covers 1849-2150 (~32 MB, gitignored)
download "$NAIF/spk/planets/de440s.bsp" "$DATA_DIR/spk/de440s.bsp"

echo "Done."
