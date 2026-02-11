# vda5050_message_structs

This Project contains C++17 structs representing the all VDA5050 message objects.
All structures were designed to match the VDA5050 v2.1.0 Interface for the communication
between automated guided vehicles (AGV) and a master control.

## (Optional) Installation

```sh
cd vda5050_messages_structs
mkdir build
cd build
cmake .. -G Unix\ Makefiles -DCMAKE_INSTALL_PREFIX=<your install prefix>
make build
ctest # optional if tests were built
make install
```

## Including

If installed, add to your CMakeLists.txt:

```cmake
find_package(vda5050_message_structs)
target_add_library(your_target vda5050_message_structs)
```

Otherwise copy this project into a subdirectory and add to your CMakeLists.txt:

```cmake
add_subdirectory(<path_to_subdirectory>/vda5050_message_structs)
target_add_library(your_target vda5050_message_structs)
```

---
_Note: Naming scheme does not follow SELE Guidelines, to match VDA5050 names._