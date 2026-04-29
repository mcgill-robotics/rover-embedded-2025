git submodule update --init --recursive
cp mpack_cmake.txt libs/mpack/CMakeLists.txt
cd libs/mpack/
./tools/amalgamate.sh