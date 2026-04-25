REM this is to manually setup the depencies if you arent using cmake
REM when using cmake the cmake files should install everything for you
@echo off
echo "Cloning submodules"
git submodule update --init --recursive
echo "Creating mpack amalgamation directory"
mkdir libs\mpack\
echo "Copying CMakeLists.txt for mpack"
copy src\mpack_cmake.txt libs\mpack\CMakeLists.txt
echo "Manually downloading amalgamation package from github as tar.gz because this is on windows"
curl.exe -L -o libs\mpack\mpack.tar.gz --url https://github.com/ludocode/mpack/releases/download/v1.1.1/mpack-amalgamation-1.1.1.tar.gz
tar -xzf libs\mpack\mpack.tar.gz -C libs/mpack --strip-components=1
DEL libs\mpack\mpack.tar.gz
