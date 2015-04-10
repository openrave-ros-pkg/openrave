###########################################
# textserver openrave plugin
###########################################
add_library(irp6kinematic SHARED irp6kinematic.cpp irp6kinematic.h plugindefs.h)

if( MSVC )
  target_link_libraries(irp6kinematic libopenrave imm32 winmm ws2_32)
else()
  target_link_libraries(irp6kinematic libopenrave)
endif()

set_target_properties(irp6kinematic PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS irp6kinematic DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-irp6kinematic)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-TEXTSERVER_DISPLAY_NAME "Plugin for a text socket-based server" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-irp6kinematic PARENT_SCOPE)
