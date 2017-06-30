#examples
cmake_minimum_required (VERSION 2.6)
project (libkindrv_exampels)

# LIBKINDRV entries should only be shown in advanced view
mark_as_advanced(LIBKINDRV_LIB LIBKINDRV_INCLUDE_DIR)

# find installed kindrv library and include dir
set (LIBKINDRV_FOUND FALSE CACHE STRING "FALSE" FORCE)

include (FindPkgConfig OPTIONAL)
if (PKG_CONFIG_FOUND)
  # find libkindrv with pkg-config (recommended)
  pkg_check_modules (LIBKINDRV REQUIRED libkindrv)

else (PKG_CONFIG_FOUND)
  message (WARNING "FindPkgConfig not found, cannot use pkg-config to detect"
                   " libkindrv. Using custom cmake code to find libkinrv"
                   " and adding boost dependency (not recommended)")

  # find libkindrv
  # no cmake module available yet, therefore these lines instead of
  #  simply using "find_package(libkindrv REQUIRED)"
  find_library (LIBKINDRV_LIBRARIES kindrv HINTS ENV LD_LIBRARY_PATH)
  find_path (LIBKINDRV_INCLUDE_DIRS kindrv.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
    HINTS
      ENV CPATH
    PATH_SUFFIXES
      libkindrv
  )

  if (LIBKINDRV_LIBRARIES AND LIBKINDRV_INCLUDE_DIRS)
    set (LIBKINDRV_FOUND TRUE CACHE STRING "TRUE" FORCE)
  endif (LIBKINDRV_LIBRARIES AND LIBKINDRV_INCLUDE_DIRS)

  # find boost-system
  find_package (Boost REQUIRED COMPONENTS system)
  set (LIBKINDRV_LIBRARIES ${LIBKINDRV_LIBRARIES} ${Boost_LIBRARIES})
  set (LIBKINDRV_INCLUDE_DIRS ${LIBKINDRV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
endif (PKG_CONFIG_FOUND)

if (LIBKINDRV_FOUND)
  include_directories (${LIBKINDRV_INCLUDE_DIRS})
  if (LIBKINDRV_LIBRARY_DIRS)
    link_directories (${LIBKINDRV_LIBRARY_DIRS})
  endif (LIBKINDRV_LIBRARY_DIRS)
else (LIBKINDRV_FOUND)
  message(FATAL_ERROR "Could not find libkindrv!")
endif (LIBKINDRV_FOUND)

find_package(Threads REQUIRED)

add_executable (libkindrv_ex_ctrl ex_ctrl.cpp)
add_executable (libkindrv_ex_retract ex_retract.cpp)
add_executable (libkindrv_ex_home ex_home.cpp)
add_executable (libkindrv_ex_finger ex_finger.cpp)
add_executable (libkindrv_ex_move_fb ex_move_fb.cpp)
add_executable (libkindrv_ex_move_fblrud ex_move_fblrud.cpp)
add_executable (libkindrv_ex_joystick ex_joystick.cpp)
add_executable (arm_control_keyboard arm_control_keyboard.cpp)
add_executable (arm_control_joystick arm_control_joystick.cpp)
add_executable (emg_classification emg_classification.cpp GUSSS.cpp ads1256.cpp spi.cpp)
add_executable (emg_arm_control emg_arm_control.cpp GUSSS.cpp ads1256.cpp spi.cpp)
add_executable (emg_arm_control_network emg_arm_control_network.cpp GUSSS.cpp ads1256.cpp spi.cpp)
add_executable (get_training_parameters get_training_parameters.cpp GUSSS.cpp ads1256.cpp spi.cpp)

target_link_libraries (libkindrv_ex_ctrl ${LIBKINDRV_LIBRARIES})
target_link_libraries (libkindrv_ex_finger ${LIBKINDRV_LIBRARIES})
target_link_libraries (libkindrv_ex_retract ${LIBKINDRV_LIBRARIES})
target_link_libraries (libkindrv_ex_move_fb ${LIBKINDRV_LIBRARIES})
target_link_libraries (libkindrv_ex_move_fblrud ${LIBKINDRV_LIBRARIES})
target_link_libraries (libkindrv_ex_home ${LIBKINDRV_LIBRARIES})
target_link_libraries (libkindrv_ex_joystick ${LIBKINDRV_LIBRARIES})
target_link_libraries (arm_control_keyboard ${LIBKINDRV_LIBRARIES} Threads::Threads)
target_link_libraries (arm_control_joystick ${LIBKINDRV_LIBRARIES} Threads::Threads)
target_link_libraries (emg_arm_control ${LIBKINDRV_LIBRARIES} Threads::Threads itpp wiringPi bcm2835)
target_link_libraries (emg_arm_control_network itpp wiringPi bcm2835)
target_link_libraries (get_training_parameters itpp wiringPi bcm2835)
target_link_libraries (emg_classification itpp wiringPi bcm2835)
