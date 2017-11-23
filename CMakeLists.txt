cmake_minimum_required(VERSION 3.5)
project(csci251_project3)

set(CMAKE_C_STANDARD 99)

# Include P-Thread libraries
#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)

set(SOURCE_FILES src/main.c src/utils/safemalloc.h src/utils/safemalloc.c src/robot.c src/robot.h src/simulation.c src/simulation.h)
add_executable(main ${SOURCE_FILES})

# link targets with the thread libraries
#target_link_libraries(main Threads::Threads)
