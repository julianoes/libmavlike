#!/usr/bin/env bash

set -e

# Parse command line arguments
BUILD_TYPE="Debug"
for arg in "$@"; do
    case $arg in
        --release)
            BUILD_TYPE="Release"
            ;;
        --help|-h)
            echo "Usage: $0 [--release]"
            echo "  --release    Build in Release mode (default: Debug)"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "Building with CMAKE_BUILD_TYPE=${BUILD_TYPE}"

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
install_dir="${script_dir}/deps/install"

mkdir -p deps
pushd deps > /dev/null
if [ ! -d "rapidxml" ]; then
    git clone https://github.com/timniederhausen/rapidxml.git
fi
pushd rapidxml > /dev/null
git fetch
git checkout ce0fb9a
cmake -Bbuild -S. \
    -DRAPIDXML_BUILD_TESTS=OFF -DRAPIDXML_INSTALL=ON \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${install_dir}"
cmake --build build --target install
popd > /dev/null

# Build and install picosha2
if [ ! -d "picosha2" ]; then
    git clone https://github.com/julianoes/PicoSHA2.git picosha2
fi
pushd picosha2 > /dev/null
git fetch
git checkout cmake-install-support
cmake -Bbuild -S. \
    -DPICOSHA2_TEST=OFF -DPICOSHA2_EXAMPLE=OFF \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${install_dir}"
cmake --build build --target install
popd > /dev/null
popd > /dev/null

cmake -Bbuild -S. -DCMAKE_PREFIX_PATH="${install_dir}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build build
