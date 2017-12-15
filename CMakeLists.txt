project(PCLPlugins)
if (NOT ParaView_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.0)
    set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")
    find_package(ParaView REQUIRED)
    include(${PARAVIEW_USE_FILE})
    find_package(PythonLibs REQUIRED)
endif ()
set(CMAKE_CXX_STANDARD 14)
option(USE_EXTERNAL_PCL "Use external system wide PCL version")

#if(NOT USE_EXTERNAL_PCL)
#set(VTK_DIR /usr/lib/cmake/paraview-5.4)
#set(ParaView_CMAKE_DIR /usr/lib/cmake/paraview-5.4)
#set(PCL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pcl/build)
#endif()

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${VTK_PYTHON_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_definitions(${PCL_DEFINITIONS})


set(gui_sources)
set(gui_interfaces)
set(gui_xml)
set(gui_deps)
set(servermanager_sources)
set(servermanager_xml)
set(server_deps)
set(IFACES)
set(IFACE_SRCS)

QT5_WRAP_CPP(MOC_SRCS pqPCLOpenNI2SourceWidget.h)


add_paraview_property_group_widget(IFACES IFACE_SRCS
        TYPE "PCL_OPENNI2_Source_Widget"
        CLASS_NAME pqPCLOpenNI2SourceWidget)



list(APPEND servermanager_sources
        vtkAnnotateOBBs.h
        vtkAnnotateOBBs.cxx
        vtkPCLConversions.h
        vtkPCLConversions.cxx
        vtkPCLEuclideanClusterExtraction.h
        vtkPCLEuclideanClusterExtraction.cxx
        #  vtkPCLNDTRegistration.cxx
        #  vtkPCLNormalEstimation.cxx
        #  vtkPCLRadiusOutlierRemoval.cxx
        vtkPCDReader.h
        vtkPCDReader.cxx
        vtkPCDWriter.h
        vtkPCDWriter.cpp
        #  vtkPCLSACSegmentationCylinder.cxx
        vtkPCLSACSegmentationPlane.h
        vtkPCLSACSegmentationPlane.cxx
        vtkPCLVoxelGrid.h
        vtkPCLVoxelGrid.cxx
        vtkPCLOpenNI2Source.h
        vtkPCLOpenNI2Source.cxx
        )
#if (USE_OPENNI)
#    list(APPEND servermanager_sources
#            vtkPCLOpenNISource.h
#            vtkPCLOpenNISource.cxx
#            )
#endif ()

list(APPEND servermanager_xml
        xml/vtkAnnotateOBBs.xml
        xml/vtkPCLEuclideanClusterExtraction.xml
        #xml/vtkPCLNDTRegistration.xml
        #xml/vtkPCLNormalEstimation.xml
        #xml/vtkPCLRadiusOutlierRemoval.xml
        xml/vtkPCDReader.xml
        #xml/vtkPCLSACSegmentationCylinder.xml
        xml/vtkPCLSACSegmentationPlane.xml
        xml/vtkPCLVoxelGrid.xml
        xml/vtkPCDWriter.xml
        xml/vtkPCLOpenNI2Source.xml
        #xml/vtkThresholdPoints.xml
        )

#if(USE_OPENNI)
#list(APPEND servermanager_xml
#
#  )
#endif()

#list(APPEND gui_xml
#  xml/PCLFiltersGUI.xml
#  )

#if(USE_OPENNI)
#list(APPEND gui_xml
#
#  )
#endif()

list(APPEND server_deps
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
        )

set(library_name vtkPCLFilters)
include(GenerateExportHeader)

add_paraview_plugin(${library_name} "1.0"
        REQUIRED_ON_SERVER
        REQUIRED_ON_CLIENT
        SERVER_MANAGER_XML ${servermanager_xml}
        SERVER_MANAGER_SOURCES ${servermanager_sources}
        GUI_INTERFACES ${IFACES}
        SOURCES ${MOC_SRCS} ${IFACE_SRCS} pqPCLOpenNI2SourceWidget.h pqPCLOpenNI2SourceWidget.cpp
        )


generate_export_header(${library_name} EXPORT_FILE_NAME ${library_name}Module.h)


target_link_libraries(${library_name} LINK_PUBLIC ${gui_deps} ${server_deps})

if (VTK_WRAP_PYTHON)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/wrap-python.cmake)
    wrap_python(${library_name} "${servermanager_sources}" ${VTK_LIBRARIES})
endif ()


if (BUILD_TESTING)
    add_subdirectory(Testing)
endif ()
