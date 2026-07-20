# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1"
  "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-build"
  "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix"
  "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/tmp"
  "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp"
  "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src"
  "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/uos-robotics/ws_livox/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp${cfgdir}") # cfgdir has leading slash
endif()
