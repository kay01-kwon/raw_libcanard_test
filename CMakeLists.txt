cmake_minimum_required(VERSION 3.8)
project(raw_libcanard_test)

set(CMAKE_CXX_STANDARD 17)

# CANARD sources
set(CANARD_INCLUDE 
include/canard_impl)

set(CANARD_SRC
  ${CANARD_INCLUDE}/canard_internals/canard.c
  ${CANARD_INCLUDE}/canard_interface/canard_interface.cpp
  ${CANARD_INCLUDE}/driver/socketcan.c
)
# DSDL generated sources
set(DSDL_GENERATED_SRC
    include/dsdl_generated/uavcan.protocol.NodeStatus.c
    include/dsdl_generated/uavcan.protocol.GetNodeInfo_res.c
    include/dsdl_generated/uavcan.protocol.GetNodeInfo_req.c
    include/dsdl_generated/uavcan.equipment.esc.RawCommand.c
    include/dsdl_generated/uavcan.equipment.esc.RPMCommand.c
    include/dsdl_generated/uavcan.equipment.esc.Status.c
    include/dsdl_generated/uavcan.protocol.dynamic_node_id.Allocation.c
    include/dsdl_generated/uavcan.protocol.param.GetSet_req.c
    include/dsdl_generated/uavcan.protocol.param.GetSet_res.c
    include/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_req.c
    include/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_res.c
    include/dsdl_generated/uavcan.protocol.RestartNode_res.c
    include/dsdl_generated/uavcan.protocol.RestartNode_req.c
)

# Compiler flags
add_definitions(-DDRONECAN_CXX_WRAPPERS)

include_directories(
  include
  ${CANARD_INCLUDE}
  ${CANARD_INCLUDE}/canard_internals
  ${CANARD_INCLUDE}/driver
  ${CANARD_INCLUDE}/canard_interface
  ${CANARD_INCLUDE}/canard
  include/dsdl_generated
)

add_library(canard_lib
  ${CANARD_SRC}
)

add_library(dsdl_generated_lib
  ${DSDL_GENERATED_SRC}
)

add_executable(esc_test_main
  src/esc_test_main.cpp
  include/esc_test/esc_test.cpp
)

target_link_libraries(esc_test_main PRIVATE canard_lib dsdl_generated_lib)