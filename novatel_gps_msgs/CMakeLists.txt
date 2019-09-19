cmake_minimum_required(VERSION 2.8.3)
project(novatel_gps_msgs)

set(MSG_DEPS 
  std_msgs)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

add_service_files(
  FILES
  NovatelFRESET.srv
)

add_message_files(DIRECTORY msg FILES
  ClockSteering.msg
  Gpgga.msg
  Gpgsa.msg
  Gpgsv.msg
  Gphdt.msg
  Gprmc.msg
  Inscov.msg
  Inspva.msg
  Inspvax.msg
  Insstdev.msg
  NovatelCorrectedImuData.msg
  NovatelDualAntennaHeading.msg
  NovatelExtendedSolutionStatus.msg
  NovatelHeading2.msg
  NovatelMessageHeader.msg
  NovatelPosition.msg
  NovatelReceiverStatus.msg
  NovatelSignalMask.msg
  NovatelUtmPosition.msg
  NovatelVelocity.msg
  NovatelXYZ.msg
  Range.msg
  RangeInformation.msg
  Satellite.msg
  Time.msg
  Trackstat.msg
  TrackstatChannel.msg
)
generate_messages(DEPENDENCIES ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)


# Symbolic link to the json file for YouCompleteMe
set(symtarget ${CATKIN_DEVEL_PREFIX}/../build/compile_commands.json)
set(symlink ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json)
ADD_CUSTOM_TARGET(link_target_msgs ALL
	                  COMMAND ${CMAKE_COMMAND} -E create_symlink ${symtarget} ${symlink})

