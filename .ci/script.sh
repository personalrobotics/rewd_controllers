#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

export PACKAGE_NAMES="$(./scripts/internal-get-packages.py distribution.yml ${REPOSITORY})"

./scripts/internal-build.sh ${PACKAGE_NAMES}
