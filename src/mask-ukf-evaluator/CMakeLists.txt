#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME mask-ukf-evaluator)

# Eigen
find_package(Eigen3 REQUIRED)

# nanoflann
find_package(nanoflann QUIET)

set(${EXE_TARGET_NAME}_HDR
    include/Metric.h
    include/MetricAddS.h
    include/MetricEmpty.h
    include/MetricRMS.h
    include/MetricRMSSymmetry.h
    include/ObjectResults.h
    include/PointCloudAdaptor.h
    include/Results.h
    include/ResultsDenseFusionADDS.h
    include/ResultsPoseRBPFADDS.h
    include/SingleVideoResult.h
    include/kdTree.h
)

set(${EXE_TARGET_NAME}_SRC
    src/Metric.cpp
    src/MetricAddS.cpp
    src/MetricEmpty.cpp
    src/MetricRMS.cpp
    src/MetricRMSSymmetry.cpp
    src/ObjectResults.cpp
    src/SingleVideoResult.cpp
)

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_HDR}
               ${${EXE_TARGET_NAME}_SRC}
               src/main.cpp
)

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      Eigen3::Eigen
                      nanoflann::nanoflann
)
