# this is to manually setup the depencies if you arent using cmake
# when using cmake the cmake files should install everything for you
git submodule update --init --recursive
cp src/mpack_cmake.txt libs/mpack/CMakeLists.txt
cd libs/mpack_git/
./tools/amalgamate.sh
cp .build/amalgamation ../mpack