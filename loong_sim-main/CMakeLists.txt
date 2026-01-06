#[[
/*
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============

=====================================================]]

cmake_minimum_required(VERSION 3.0)
project(sim)
set(CMAKE_BUILD_TYPE "Release")
if(MSVC)
	# add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
	add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
elseif(UNIX)
	add_definitions(-std=c++17)
endif()
# ================================
include(../loong_utility/loong_utility.cmake)
set(allInc "module" "src")
# 第三方.h
set(incPreFix ${CMAKE_SOURCE_DIR}/third_party/include)
file(GLOB children RELATIVE ${incPreFix} ${incPreFix}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${incPreFix}/${child})
		list(APPEND allInc ${incPreFix}/${child})
	endif()
endforeach()
# 项目.h
set(srcPreFix ${CMAKE_SOURCE_DIR}/src)
file(GLOB children RELATIVE ${srcPreFix} ${srcPreFix}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${srcPreFix}/${child})
		list(APPEND allInc ${srcPreFix}/${child})
	endif()
endforeach()

include_directories(${allInc})

# ================================
set(simLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("=== linux a64架构 ===")
	link_directories("third_party/lib_lin_a64")
	set(nabo nabo_a64)
	set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../third_party/lib_lin_a64;")
else()
	message("=== linux x64架构 ===")
	link_directories("third_party/lib_lin_x64")
	set(nabo nabo_x64)
	set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../third_party/lib_lin_x64;")
endif()
# ================================
# -----------
file(GLOB srcFiles src/*.cpp)
add_library(sim STATIC ${srcFiles})
target_link_libraries(sim util mujoco glfw3 dl pthread)
# -----------
file(GLOB_RECURSE srcFiles "main/ctrl_sim_loco/*.cpp")
add_executable(main_loco ${srcFiles})
target_link_directories(main_loco PRIVATE module/nabo_locomotion)
target_link_libraries(main_loco sim ${nabo})

# ===============================
set(CMAKE_INSTALL_PREFIX ../nabo_output)
install(TARGETS  DESTINATION .)