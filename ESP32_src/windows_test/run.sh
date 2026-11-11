#!/usr/bin/env bash
# Windows Git Bash entry; protocol and serial I/O are handled by Python.
set -e
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_python="$task_dir/windows_test.py"
if command -v cygpath >/dev/null 2>&1; then
  task_python="$(cygpath -w "$task_python")"
fi
if command -v py >/dev/null 2>&1 && py -3 -c 'import sys; assert sys.version_info >= (3, 9)' 2>/dev/null; then
  exec py -3 "$task_python" "$@"
fi
for task_command in python python3; do
  if command -v "$task_command" >/dev/null 2>&1 && "$task_command" -c 'import sys; assert sys.version_info >= (3, 9)' 2>/dev/null; then
    exec "$task_command" "$task_python" "$@"
  fi
done
echo 'Please install Python 3.9+ from python.org (include pip), then run again.' >&2
exit 1
