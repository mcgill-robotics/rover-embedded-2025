git submodule update --init --recursive
cp mpack_cmake.txt libs/mpack/CMakeLists.txt
cd libs/mpack_git/
./tools/amalgamate.sh
cp .build/amalgamation ../mpack