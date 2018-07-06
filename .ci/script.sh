#!/usr/bin/env bash

set -ex

./scripts/internal-build.sh ${PACKAGE_NAMES}
./scripts/internal-test.sh ${PACKAGE_NAMES}
