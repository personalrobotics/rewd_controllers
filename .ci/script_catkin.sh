#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

export PACKAGE_NAMES="$(./scripts/internal-get-packages.py distribution.yml ${REPOSITORY})"
./scripts/internal-build.sh ${PACKAGE_NAMES}

if [ $BUILD_NAME = DOCS ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/build_docs.sh"
  exit 0
fi

# Check code style
if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args check-format -- rewd_controllers
fi

# Manually build rewd_controllers's tests; they are not built automatically because it is not a Catkin package.
if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=ON --make-args tests -- rewd_controllers
else
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=OFF --make-args tests -- rewd_controllers
fi

# Run tests and measure test coverage if CodeCov is on.
if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  ./scripts/internal-run.sh make -C build/rewd_controllers rewd_controllers_coverage
else
  ./scripts/internal-run.sh env CTEST_OUTPUT_ON_FAILURE=true make -C build/rewd_controllers test
fi
