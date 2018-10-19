# coreSLAM

A lightweight SLAM Algorithm in C++ based on Center of Robotics research.

## Reference:

A SLAM Algorithm in less than 200 lines of C code by Bruno Steux, Oussama El Hamzaoui
Mines ParisTech - Center of Robotics, Paris, FRANCE.

http://www.researchgate.net/publication/228374722_CoreSLAM_a_SLAM_Algorithm_in_less_than_200_lines_of_C_code

![alt text](https://github.com/WestTeam/CoreSLAM/blob/master/images/demo.png)

## Build

You need to install CMake with version higher than 2.8.1.

```
cd PATH-TO-CoreSLAM
mkdir build
cd build
cmake ../
make
```

### Note:
The CMakeLists is written for macOS build. If you want to build it on Linux just change:

```
target_link_libraries(
    TestLabReverse
	${OpenCV_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/build/libCoreSLAM.dylib )
```
To,
```
target_link_libraries(
    TestLabReverse
	${OpenCV_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/build/libCoreSLAM.so )
```
