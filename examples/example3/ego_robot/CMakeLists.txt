

find_package (Threads)
include_directories(
    /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include
)
add_definitions( -DPRT_PLAT_LINUXUSER )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-variadic-macros -Wno-unused-parameter -Wno-write-strings -Wno-vla -Wno-cast-function-type -Wno-unused-variable -Wno-reorder ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-variadic-macros -Wno-unused-parameter -Wno-write-strings -Wno-vla -Wno-cast-function-type -Wno-pointer-sign -Wno-unused-variable -Wno-incompatible-pointer-types -Wno-unused-label -Wno-missing-braces ")

find_package(ompl)

###############################################################################
# Macro
###############################################################################

macro(kobuki_core_add_demo name)
  set(target_name kobuki_core_demo_${name})
  add_executable(${target_name}
  ${name}.cpp
  ego_robot.h
  ego_robot.c
  helper.hpp
  helper.cpp
  monitor.hpp
  monitor.cpp
  socket.hpp
  socket.cpp
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/ext_compat.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/libhandler.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/libhandler-internal.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/Prt.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/PrtConfig.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/PrtExecution.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/PrtLinuxUserConfig.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/PrtProgram.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/PrtTypes.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/PrtValues.h
  /home/ubuntu/P-RTA/P/Bld/Drops/Prt/include/sal.h
  )
  target_link_libraries(${target_name}
    ${PROJECT_NAME}
    ecl_command_line::ecl_command_line
    ecl_console::ecl_console
    /home/ubuntu/P-RTA/P/Bld/Drops/Prt/lib/libPrt_static.a
    /home/ubuntu/P-RTA/P/Ext/libhandler/out/gcc-arm-linux-gnueabihf/debug/libhandler.a
    ${CMAKE_THREAD_LIBS_INIT}
    ${OMPL_LIBRARIES}
    ${OMPLAPP_LIBRARIES}
  )
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME demo_${name})
  install(
    TARGETS
      ${target_name}
    RUNTIME DESTINATION
      lib/${PROJECT_NAME}
  )
endmacro()

###############################################################################
# Tests
###############################################################################

kobuki_core_add_demo(ego_robot_main)

