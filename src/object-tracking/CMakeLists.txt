#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME mask-ukf-object-tracker)

# Bayes Filters
find_package(BayesFilters REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             os
)

# armadillo
find_package(Armadillo REQUIRED)

# mlpack
find_package(MLPACK REQUIRED)

# nanoflann
find_package(nanoflann QUIET)

set(${EXE_TARGET_NAME}_HDR
    include/Camera.h
    include/Correction.h
    include/CorrectionICP.h
    include/DiscretizedKinematicModel.h
    include/InitGroundTruth.h
    include/MaskSegmentation.h
    include/ObjectMeasurements.h
    include/ObjectPointCloudPrediction.h
    include/PointCloudAdaptor.h
    include/PointCloudModel.h
    include/PointCloudPrediction.h
    include/PointCloudSegmentation.h
    include/StaticPrediction.h
    include/SUKFCorrection.h
    include/UKFFilter.h
    include/YCBVideoCamera.h
    include/kdTree.h
)

set(${EXE_TARGET_NAME}_SRC
    src/Camera.cpp
    src/Correction.cpp
    src/CorrectionICP.cpp
    src/DiscretizedKinematicModel.cpp
    src/InitGroundTruth.cpp
    src/MaskSegmentation.cpp
    src/ObjectMeasurements.cpp
    src/ObjectPointCloudPrediction.cpp
    src/PointCloudModel.cpp
    src/PointCloudSegmentation.cpp
    src/StaticPrediction.cpp
    src/SUKFCorrection.cpp
    src/UKFFilter.cpp
    src/YCBVideoCamera.cpp
    src/main.cpp
)

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
)

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${ARMADILLO_INCLUDE_DIRS}
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${MLPACK_INCLUDE_DIRS}
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      BayesFilters::BayesFilters
                      Eigen3::Eigen
                      YARP::YARP_os
                      nanoflann::nanoflann
                      ${ARMADILLO_LIBRARIES}
                      ${MLPACK_LIBRARIES}
                      ${OpenCV_LIBS}
)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE ${PCL_LIBRARIES})
target_include_directories(${EXE_TARGET_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${EXE_TARGET_NAME} PRIVATE ${PCL_DEFINITIONS})
target_compile_options(${EXE_TARGET_NAME} PRIVATE ${PCL_COMPILE_OPTIONS})

# file(GLOB ${EXE_TARGET_NAME}_YCB_MODELS ${CMAKE_CURRENT_SOURCE_DIR}/models/*)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
