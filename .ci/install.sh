#!/usr/bin/env bash

set -ex

mv -r "${TRAVIS_BUILD_DIR}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}"
