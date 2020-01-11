#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  cat ./build/rewd_controllers/Testing/Temporary/LastTest.log
  cat ./build/rewd_controllers/Testing/Temporary/LastTestsFailed.log
fi
