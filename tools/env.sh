# Source this (don't execute it):   . tools/env.sh
#
# Activates ESP-IDF in the current shell. The existing IDF v5.3 Python env was
# created with macOS's Python 3.9, but `python3` on PATH is Homebrew's 3.14,
# which makes a plain `. ~/esp/esp-idf/export.sh` look for a non-existent
# idf5.3_py3.14_env. Pointing IDF_PYTHON_ENV_PATH at the working env fixes it.

export IDF_PATH="$HOME/esp/esp-idf"
export IDF_TOOLS_PATH="$HOME/.espressif"
export IDF_PYTHON_ENV_PATH="$IDF_TOOLS_PATH/python_env/idf5.3_py3.9_env"

_idf_log="$(mktemp -t idf-export)"
if . "$IDF_PATH/export.sh" > "$_idf_log" 2>&1 && command -v idf.py > /dev/null; then
    echo "$(idf.py --version 2>/dev/null) ready"
else
    echo "ESP-IDF activation FAILED. export.sh output:" >&2
    cat "$_idf_log" >&2
fi
rm -f "$_idf_log"
unset _idf_log
