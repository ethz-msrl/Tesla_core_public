#! /bin/bash
set -e
# shellcheck disable=SC1091
source /tc_ws/devel/setup.bash

exec "$@"
