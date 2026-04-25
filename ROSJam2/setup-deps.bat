git submodule update --init --recursive
mkdir libs\mpack\
copy mpack_cmake.txt libs\mpack\CMakeLists.txt
echo "skipping amalgamation of mpack library on windows (download the amalgamation package manually from here https://github.com/ludocode/mpack/releases and place its contents in libs/mpack/)"
echo "There should at least be a libs/mpack/src/mpack.c and libs/mpack/src/mpack.h for the library to work"
echo "Do not delete the CMakeLists.txt that was placed (if you did rerun this after copying the amalgamation package)"