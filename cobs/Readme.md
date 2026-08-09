# COBS (Consistent Overhead Byte Stuffing)

Implementations of 
[COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) 
in C and Python

A [`CMakeLists.txt`](./cobs-c/src/CMakeLists.txt) file  is available for the C implementation to easily 
import into C projects (ex: STM32 cmake projects in vscode)

> Do not import the `CMakeLists.txt` in `cobs-c`. Make sure to import the one at `cobs-c/src` by adding it as a subdirectory in cmake then adding the `cobs-c` target in the linked-libraries.
> See [here](cobs-c/CMakeLists.txt) for an example.