#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$BASE_DIR/SimpleFOCStudio"
exec /home/ruihu/miniconda3/bin/conda run --no-capture-output -p "$BASE_DIR/simplefoc-studio-conda" python simpleFOCStudio.py "$@"
