#!/usr/bin/env python3
"""Fix livox_ros_driver2 CMakeLists.txt for ROS2 Humble on aarch64.
Two issues:
1. Humble's rosidl_get_typesupport_target target name differs; need to set LIVOX_INTERFACE_TARGET.
2. get_target_property on that target fails at configure time (returns NOTFOUND).
   Fix: use $<TARGET_PROPERTY:...> generator expression instead.
"""
import sys

cmake_path = sys.argv[1] if len(sys.argv) > 1 else "CMakeLists.txt"

with open(cmake_path, "r") as f:
    content = f.read()

# Fix 1: set LIVOX_INTERFACE_TARGET for humble/jazzy branch
old1 = """  if(DISTRO_ROS STREQUAL "humble" OR DISTRO_ROS STREQUAL "jazzy")
    rosidl_get_typesupport_target(cpp_typesupport_target
    ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
    target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")"""

new1 = """  if(DISTRO_ROS STREQUAL "humble" OR DISTRO_ROS STREQUAL "jazzy")
    rosidl_get_typesupport_target(cpp_typesupport_target
    ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
    set(LIVOX_INTERFACE_TARGET ${cpp_typesupport_target})
    target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")"""

assert old1 in content, "Fix#1: old block not found"
content = content.replace(old1, new1)
print("Fix#1 OK: LIVOX_INTERFACE_TARGET added for humble/jazzy")

# Fix 2: replace ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES} with generator expression
old2 = "${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs"
new2 = "$<TARGET_PROPERTY:${LIVOX_INTERFACE_TARGET},INTERFACE_INCLUDE_DIRECTORIES>   # for custom msgs (gen-expr for humble compat)"

assert old2 in content, "Fix#2: old include pattern not found"
content = content.replace(old2, new2)
print("Fix#2 OK: replaced with generator expression")

with open(cmake_path, "w") as f:
    f.write(content)
print("ALL PATCHES APPLIED")
