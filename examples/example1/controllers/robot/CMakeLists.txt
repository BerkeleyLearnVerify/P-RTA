cmake_minimum_required (VERSION 3.1)
project (robot)
set(projectName robot)
find_package (Threads)
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include
    /Applications/Webots.app/include/controller/c # If your Webots Simulator is in a different directory, change this!
    ${CMAKE_CURRENT_LIST_DIR}/../../../../reelay
    ${CMAKE_CURRENT_LIST_DIR}/../../../../reelay/include
    /usr/local/include # You may not need this if your compiler can find C/C++ header files. If your compiler is having trouble finding header files and if your header files are located in another directory, change this!
    /usr/local/opt/boost/include # This is for boost headers. Boost is required by Reelay. So if you want to use Reelay with P-RTA and your boost headers are in a different firectory, change this!
)

add_definitions( -DPRT_PLAT_LINUXUSER )


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O2 -pthread ")



# Link with the Webots controller library.
link_directories(/Applications/Webots.app/lib) # If your Webots Simulator is in a different directory, change this!
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

add_executable(robot
    ${CMAKE_CURRENT_LIST_DIR}/robot.h
    ${CMAKE_CURRENT_LIST_DIR}/robot.c
    ${CMAKE_CURRENT_LIST_DIR}/helper.h
    ${CMAKE_CURRENT_LIST_DIR}/helper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/Main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../../../p_reelay/p_reelay.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../p_reelay/p_reelay.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/ext_compat.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/libhandler.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/libhandler-internal.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/Prt.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/PrtConfig.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/PrtExecution.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/PrtLinuxUserConfig.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/PrtProgram.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/PrtTypes.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/PrtValues.h
    ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/include/sal.h
)

    target_link_libraries(robot
        ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Bld/Drops/Prt/lib/libPrt_static.a
        ${CMAKE_CURRENT_LIST_DIR}/../../../../P/Ext/libhandler/out/gcc-amd64-apple-darwin20.6.0/debug/libhandler.a # If you are not using MacOS, you need to change gcc-amd64-apple-darwin20.6.0 accordingly!
        ${CMAKE_THREAD_LIBS_INIT} ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
        /usr/local/lib/libcudd.a # If your libcudd.a is in a different directory, then change this! Reelay installation will tell you where libcudd.a is located.
    )

