add_library(vl53l1x INTERFACE)

target_sources(vl53l1x INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_calibration.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_calibration.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_core.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_core.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_debug.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_debug.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_preset_modes.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_api_preset_modes.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_core.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_core.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_core_support.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_core_support.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_def.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_error_codes.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_error_exceptions.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_error_strings.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_error_strings.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_ll_def.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_ll_device.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_nvm_map.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_preset_setup.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_register_funcs.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_register_funcs.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_register_map.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_register_settings.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_register_structs.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_silicon_core.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_silicon_core.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_tuning_parm_defaults.h
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_wait.c
  ${CMAKE_CURRENT_LIST_DIR}/core/vl53l1_wait.h
  ${CMAKE_CURRENT_LIST_DIR}/platform/vl53l1_platform.c
  ${CMAKE_CURRENT_LIST_DIR}/platform/vl53l1_platform.h
  ${CMAKE_CURRENT_LIST_DIR}/platform/vl53l1_platform_log.h
  ${CMAKE_CURRENT_LIST_DIR}/platform/vl53l1_platform_user_config.h
  ${CMAKE_CURRENT_LIST_DIR}/platform/vl53l1_platform_user_data.h
  ${CMAKE_CURRENT_LIST_DIR}/platform/vl53l1_platform_user_defines.h
  ${CMAKE_CURRENT_LIST_DIR}/platform/vl53l1_types.h )

target_include_directories(vl53l1x INTERFACE ${CMAKE_CURRENT_LIST_DIR}/core)
target_include_directories(vl53l1x INTERFACE ${CMAKE_CURRENT_LIST_DIR}/platform)
target_link_libraries(vl53l1x INTERFACE pico_stdlib hardware_i2c)
