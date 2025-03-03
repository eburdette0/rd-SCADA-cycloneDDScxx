cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(rd-SCADA-cyclonedds-cxx VERSION 0.1 LANGUAGES C CXX)

option(WERROR OFF)


#control target platform options
option(REMOTE_TARGET "" OFF)
option(REALDATA "" OFF)
option(HARDWARE_UEI "" OFF)
option(HARDWARE_LABJACK "" OFF)

#MUST RECONFIGURE TO REGENERATE HEADER FILE!
configure_file(systemDifferences.h.in ${CMAKE_SOURCE_DIR}/include/systemDifferences.h)

if(NOT REMOTE_TARGET) #cycloneDDS should be installed in lib of remote target directroy
  set(CMAKE_PREFIX_PATH ~/CProjects/installs/cycloneDDS_linux_x86/lib/cmake)
endif()

find_package(Boost 1.61)

message("${CMAKE_PREFIX_PATH}")
if(NOT TARGET CycloneDDS-CXX::ddscxx)
  find_package(CycloneDDS-CXX REQUIRED)
endif()





add_subdirectory(include/FTXUI)

set(PROJECT_SOURCES
        main.cpp
        ./include/hardwareInterface/getdata.cpp
        ./include/hardwareInterface/getdata.h
        ./include/FIR/FIR_ds.cpp
        ./include/FIR/FIR_ds.h
        ./include/PID/PID_V1.cpp
        ./include/PID/PID_V1.h
        ./include/rigSim/rigSim.cpp
        ./include/rigSim/rigSim.h
        ./include/csvIO/datasaver.hpp
        ./include/networkData/networkData.h
        ./include/networkData/networkData.cpp
        ./include/rapidjson/rapidjson.h
        ./include/display/tui.hpp
        ./include/decimation/decimator.h
        ./include/decimation/decimator.cpp
        ./include/DDSsources/networkDataStructures.cpp
        ./include/DDSsources/networkDataStructures.hpp
        ./include/physicalCalculations/physicalCalculations_V1.cpp
        ./include/physicalCalculations/physicalCalculations_V1.h
        ./include/definitions.h
        ./include/macro_definitions.h
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include) #to reference in the include folder without typing it out every time


add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})


target_link_libraries(${PROJECT_NAME} 
                                    PRIVATE ftxui::screen
                                    PRIVATE ftxui::dom
                                    PRIVATE ftxui::component)

target_link_libraries(${PROJECT_NAME} PRIVATE CycloneDDS-CXX::ddscxx) 

# if (not REMOTE_TARGET)
#   idlcxx_generate(TARGET networkDataStructures FILES "networkDataStructures.idl" WARNINGS no-implicit-extensibility)
#   target_link_libraries(${PROJECT_NAME} PRIVATE networkDataStructures)
# endif()

if (REALDATA)
  if (HARDWARE_UEI)
    target_link_libraries(${PROJECT_NAME} PRIVATE powerdna)

  endif()
  if (HARDWARE_LABJACK)
    target_link_libraries(${PROJECT_NAME} PRIVATE labjackusb)
  endif()
endif()


# #VERY IMPORTANT -- WIN32_EXECUTABLE FALSE to print to console
# set_target_properties(${PROJECT_NAME} PROPERTIES
#     MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
#     MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
#     MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
#     MACOSX_BUNDLE TRUE
#     WIN32_EXECUTABLE FALSE
# )

# install(TARGETS ${PROJECT_NAME}
#     BUNDLE DESTINATION .
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )
