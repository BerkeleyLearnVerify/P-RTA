cmake_minimum_required (VERSION 3.1)
project (slave)
set(projectName slave)
find_package (Threads)
include_directories(
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include
    /Applications/Webots.app/include/controller/c
    /Users/beyazit/Documents/reelay
    /Users/beyazit/Documents/reelay/include
    /usr/local/include
    /usr/local/opt/boost/include)

add_definitions( -DPRT_PLAT_LINUXUSER )


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O2 -pthread ")



# Link with the Webots controller library.
link_directories(/Applications/Webots.app/lib/controller)
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

add_executable(slave
    slave.h
    slave.c
    helper.h
    helper.cpp
    Main.cpp
    /Users/beyazit/Documents/SOTER2.0/p_reelay/p_reelay.h
    /Users/beyazit/Documents/SOTER2.0/p_reelay/p_reelay.cpp
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/ext_compat.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/libhandler.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/libhandler-internal.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/Prt.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/PrtConfig.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/PrtExecution.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/PrtLinuxUserConfig.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/PrtProgram.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/PrtTypes.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/PrtValues.h
    /Users/beyazit/Documents/P/Bld/Drops/Prt/include/sal.h)

    target_link_libraries(slave /Users/beyazit/Documents/P/Bld/Drops/Prt/lib/libPrt_static.a /Users/beyazit/Documents/P/Ext/libhandler/out/gcc-amd64-apple-darwin20.6.0/debug/libhandler.a ${CMAKE_THREAD_LIBS_INIT} ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} /usr/local/lib/libcudd.a)




