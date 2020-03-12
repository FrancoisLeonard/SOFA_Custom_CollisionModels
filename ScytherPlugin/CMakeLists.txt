cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

set(SCYTHER_NAME ScytherPlugin)

project(${SCYTHER_NAME})

set(SCYTHER_MAJOR_VERSION 0)
set(SCYTHER_MINOR_VERSION 1)
set(SCYTHER_VERSION ${SCYTHER_MAJOR_VERSION}.${SCYTHER_MINOR_VERSION})

set(SCYTHER_DIR src/)

find_package(SofaFramework REQUIRED)
find_package(SofaBase REQUIRED)
find_package(SofaGeneral REQUIRED)

sofa_find_package(TinyXML REQUIRED)

#if(WIN32)
#    find_package(Boost REQUIRED COMPONENTS regex chrono iostreams)
#else()
#    find_package(Boost REQUIRED COMPONENTS regex chrono iostreams)
#endif()

set(HEADER_FILES
    scyther.h
    ${SCYTHER_DIR}/ScytherDetection.h
    ${SCYTHER_DIR}/ScytherDetection.inl
    ${SCYTHER_DIR}/ScytherIntersection.h

    ${SCYTHER_DIR}/ScytherCubeModel.h
    ${SCYTHER_DIR}/ScytherPointModel.h
    ${SCYTHER_DIR}/ScytherPointModel.inl
    ${SCYTHER_DIR}/ScytherTriangleModel.h
    ${SCYTHER_DIR}/ScytherTriangleModel.inl

)

set(SOURCE_FILES
    initScytherPlugin.cpp
    ${SCYTHER_DIR}/ScytherDetection.cpp
    ${SCYTHER_DIR}/ScytherIntersection.cpp

    ${SCYTHER_DIR}/ScytherCubeModel.cpp
    ${SCYTHER_DIR}/ScytherPointModel.cpp
    ${SCYTHER_DIR}/ScytherTriangleModel.cpp
)

configure_file(scyther.h.in ${SCYTHER_NAME}/scyther.h)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${SCYTHER_NAME}/scyther.h DESTINATION "include/${SCYTHER_NAME}")

set(SCYTHER_COMPILER_FLAGS "${SCYTHER_COMPILER_FLAGS} -DSOFA_BUILD_SCYTHER_PLUGIN")
set(SCYTHER_INSTALL_DIR share/sofa/plugins/${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} )
target_link_libraries(${PROJECT_NAME} PUBLIC SofaCore SofaGuiCommon SofaGuiQt SofaBaseVisual SofaBaseCollision SofaGeneralMeshCollision SofaGeneralRigid SofaGeneralVisual SofaTopologyMapping SofaDeformable SofaBoundaryCondition SofaGraphComponent)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SCYTHER_DIR}>")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${SCYTHER_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SCYTHER_COMPILER_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

install(TARGETS ${PROJECT_NAME}
        COMPONENT ${SCYTHER_NAME}_libraries
        EXPORT ${SCYTHER_NAME}Targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION "include/${PROJECT_NAME}")

install(DIRECTORY scenes/ DESTINATION ${SCYTHER_INSTALL_DIR}/scenes)
install(DIRECTORY scenes/data/ DESTINATION ${SCYTHER_INSTALL_DIR}/scenes/data)
