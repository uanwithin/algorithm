add_library(cartpole_simulation STATIC simulation.cc controller.cc)
target_link_libraries(cartpole_simulation PUBLIC algorithms::mujoco algorithms::eigen)
target_include_directories(cartpole_simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
configure_file(cartpole.xml cartpole.xml COPYONLY)
target_compile_definitions(cartpole_simulation PRIVATE
    CARTPOLE_MODEL="${CMAKE_CURRENT_BINARY_DIR}/cartpole.xml"
)

add_executable(mujoco_cartpole main.cc viewer.cc)
target_link_libraries(mujoco_cartpole PRIVATE cartpole_simulation algorithms::mujoco_viewer)

add_executable(mujoco_cartpole_verify verify.cc)
target_link_libraries(mujoco_cartpole_verify PRIVATE cartpole_simulation)
