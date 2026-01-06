#[[
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

set(loongUtilIncs ${CMAKE_CURRENT_LIST_DIR}/src)
set(loongUtilIncPreFix ${CMAKE_CURRENT_LIST_DIR}/third_party/include)
file(GLOB children RELATIVE ${loongUtilIncPreFix} ${loongUtilIncPreFix}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${loongUtilIncPreFix}/${child})
		list(APPEND loongUtilIncs ${loongUtilIncPreFix}/${child})
	endif()
endforeach()
include_directories(${loongUtilIncs})


if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	link_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/lib_lin_a64)
else()
	link_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/lib_lin_x64)
endif()

file(GLOB_RECURSE loongUtilFiles ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
add_library(util STATIC ${loongUtilFiles})
target_link_libraries(util quill pthread)