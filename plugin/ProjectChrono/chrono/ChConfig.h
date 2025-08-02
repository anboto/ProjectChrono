// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chrono configuration header file
//
// Automatically created during CMake configuration.
//
// =============================================================================

#ifndef CH_CONFIG_H
#define CH_CONFIG_H

// -----------------------------------------------------------------------------
// Macros specifying enabled Chrono modules
// -----------------------------------------------------------------------------

// If module CASCADE was enabled, define CHRONO_CASCADE
#undef CHRONO_CASCADE

// If module IRRLICHT was enabled, define CHRONO_IRRLICHT
#undef CHRONO_IRRLICHT

// If module VSG was enabled, define CHRONO_VSG
#undef CHRONO_VSG

// If module MATLAB was enabled, define CHRONO_MATLAB
#undef CHRONO_MATLAB

// If module PARDISO_MKL was enabled, define CHRONO_PARDISO_MKL
#undef CHRONO_PARDISO_MKL

// If module MUMPS was enabled, define CHRONO_MUMPS
#undef CHRONO_MUMPS

// If module PARSERS was enabled, define CHRONO_PARSERS
#undef CHRONO_PARSERS

// If module MULTICORE was enabled, define CHRONO_MULTICORE
#undef CHRONO_MULTICORE

// If module POSTPROCESS was enabled, define CHRONO_POSTPROCESS
#undef CHRONO_POSTPROCESS

// If module PYTHON was enabled, define CHRONO_PYTHON
#undef CHRONO_PYTHON

// If module VEHICLE was enabled, define CHRONO_VEHICLE
#undef CHRONO_VEHICLE

// If module FSI was enabled, define CHRONO_FSI
#undef CHRONO_FSI

// If module GPU was enabled, define CHRONO_GPU
#undef CHRONO_GPU

// If module SYNCHRONO was enabled, define CHRONO_SYNCHRONO
#undef CHRONO_SYNCHRONO

// If module SENSOR was enabled, define CHRONO_SENSOR
#undef CHRONO_SENSOR

// If module ROS was enabled, define CHRONO_ROS
#undef CHRONO_ROS

// If module MODAL was enabled, define CHRONO_MODAL
#undef CHRONO_MODAL

// If module FMI was enabled, define CHRONO_FMI
#undef CHRONO_FMI

// -----------------------------------------------------------------------------
// OpenMP settings
// -----------------------------------------------------------------------------

// If OpenMP is found the following define is set
//   #define CHRONO_OMP_FOUND
// Set the highest OpenMP version supported, one of:
//   #define CHRONO_OMP_VERSION "2.0"
//   #define CHRONO_OMP_VERSION "3.0"
//   #define CHRONO_OMP_VERSION "4.0"
// and define one or more of the following, as appropriate
//   #define CHRONO_OMP_20
//   #define CHRONO_OMP_30
//   #define CHRONO_OMP_40
#define CHRONO_OMP_FOUND
#define CHRONO_OMP_VERSION "4.0"
#define CHRONO_OMP_20
#define CHRONO_OMP_30
#define CHRONO_OMP_40

// If OpenMP support was enabled, define CHRONO_OPENMP_ENABLED
#define CHRONO_OPENMP_ENABLED

// -----------------------------------------------------------------------------

// If SSE support was found, then
//   #define CHRONO_HAS_SSE
// and set the SSE level support, one of the following
//   #define CHRONO_SSE_LEVEL "1.0"
//   #define CHRONO_SSE_LEVEL "2.0"
//   #define CHRONO_SSE_LEVEL "3.0"
//   #define CHRONO_SSE_LEVEL "4.1"
//   #define CHRONO_SSE_LEVEL "4.2"
// and define one or more of the following, as appropriate
//   #define CHRONO_SSE_1.0
//   #define CHRONO_SSE_2.0
//   #define CHRONO_SSE_3.0
//   #define CHRONO_SSE_4.1
//   #define CHRONO_SSE_4.2

#define CHRONO_HAS_SSE
#define CHRONO_SSE_LEVEL "4.2"
#define CHRONO_SSE_1_0
#define CHRONO_SSE_2_0
#define CHRONO_SSE_3_0
#define CHRONO_SSE_4_1
#define CHRONO_SSE_4_2

// -----------------------------------------------------------------------------

// If AVX support was found, then
//   #define CHRONO_HAS_AVX
// and set the SSE level support, one of the following
//   #define CHRONO_AVX_LEVEL "1.0"
//   #define CHRONO_AVX_LEVEL "2.0"
// and define one or more of the following, as appropriate
//   #define CHRONO_AVX_1.0
//   #define CHRONO_AVX_2.0

#define CHRONO_HAS_AVX
#define CHRONO_AVX_LEVEL "2.0"

#define CHRONO_AVX_2_0

// -----------------------------------------------------------------------------

// If NEON support was found, then
//   #define CHRONO_HAS_NEON



// -----------------------------------------------------------------------------

// If FMA support was found, then
//   #define CHRONO_HAS_FMA



// -----------------------------------------------------------------------------

// Use SIMD if available

#define CHRONO_SIMD_ENABLED

// -----------------------------------------------------------------------------

// If HDF5 was found, then

#define CHRONO_HAS_HDF5



// -----------------------------------------------------------------------------

// If CUDA is available, then
//   #define CHRONO_HAS_CUDA
//   #CHRONO_CUDA_VERSION

#undef CHRONO_HAS_CUDA
#undef CHRONO_CUDA_VERSION

// If THRUST is available, then
//   #define CHRONO_HAS_THRUST
//   #CHRONO_THRUST_VERSION

#undef CHRONO_HAS_THRUST
#undef CHRONO_THRUST_VERSION

// If CUDA is not available, force Thrust to always use the OMP backend
#ifndef CHRONO_HAS_CUDA
  #ifndef THRUST_DEVICE_SYSTEM
    #define THRUST_DEVICE_SYSTEM THRUST_DEVICE_SYSTEM_OMP
  #endif
#endif

// -----------------------------------------------------------------------------

// If Google Test and Benchmark are enabled and available, then
//   #define CHRONO_HAS_GTEST
//   #define CHRONO_HAS_GBENCHMARK




// -----------------------------------------------------------------------------

// If the Chrono multicore collision detection is available, define CHRONO_COLLISION
#undef CHRONO_COLLISION

// Always use double in custom multicore collision detection
#define USE_COLLISION_DOUBLE

// -----------------------------------------------------------------------------
// Length of the path to the Chrono source directory
// -----------------------------------------------------------------------------

#define SOURCE_PATH_SIZE 48

#if !defined(__AVX2__)			// ANBOTO
#error "Project Chrono requires AVX2 support in all project, not only in the library. Please compile with -mavx2 (GCC/Clang) or /arch:AVX2 (MSVC)."	
#endif

#endif
