macro(rosidl_generator_eus_extras BIN GENERATOR_FILES TEMPLATE_DIR)
  find_package(ament_cmake_core QUIET REQUIRED)
  find_package(rosidl_generator_type_description QUIET REQUIRED)
  ament_register_extension(
    "rosidl_generate_idl_interfaces"
    "rosidl_generator_eus"
    "rosidl_generator_eus_generate_interfaces.cmake")

  normalize_path(BIN "${BIN}")
  set(rosidl_generator_eus_BIN "${BIN}")

  normalize_path(GENERATOR_FILES "${GENERATOR_FILES}")
  set(rosidl_generator_eus_GENERATOR_FILES "${GENERATOR_FILES}")

  normalize_path(TEMPLATE_DIR "${TEMPLATE_DIR}")
  set(rosidl_generator_eus_TEMPLATE_DIR "${TEMPLATE_DIR}")
endmacro()
