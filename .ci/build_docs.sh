#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"
. devel/setup.bash

REWD_CONTROLLERS_DIR="${HOME}/workspace/src/rewd_controllers"

# For branch builds, Travis only clones that branch with a fixed depth of 50
# commits. This means that the clone knows nothing about other Git branches or
# tags. We fix this by deleting and re-cloning the full repository.
rm -rf ${REWD_CONTROLLERS_DIR}
git clone "git@github.com:${TRAVIS_REPO_SLUG}.git" ${REWD_CONTROLLERS_DIR}

# Organize into "gh-pages" directory
mkdir -p ${TRAVIS_BUILD_DIR}/gh-pages

# Initialize list of API versions
cat <<EOF > ${TRAVIS_BUILD_DIR}/gh-pages/README.md
## API Documentation

EOF

mkdir build_docs
cd build_docs

while read version; do
  # Add entry to list of API versions
  echo "* [${version}](https://personalrobotics.github.io/rewd_controllers/${version}/)" >> ${TRAVIS_BUILD_DIR}/gh-pages/README.md

  # Build documentation
  git -C ${REWD_CONTROLLERS_DIR} checkout ${version}
  rm -rf *
  cmake -DDOWNLOAD_TAGFILES=ON ${REWD_CONTROLLERS_DIR}
  make docs
  mv doxygen ${TRAVIS_BUILD_DIR}/gh-pages/${version}
done < ${TRAVIS_BUILD_DIR}/.ci/docs_versions.txt
