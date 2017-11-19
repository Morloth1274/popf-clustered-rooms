cmake_minimum_required(VERSION 2.4)

project(val)
set(CMAKE_CXX_FLAGS "-Wall -std=gnu++11 -lroscpp -lrostime -lroscpp_serialization -L/opt/ros/indigo/lib ${CMAKE_CXX_FLAGS}")

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}
                          ${CMAKE_CURRENT_SOURCE_DIR}/..
                          ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules)

set(DOXYFILE_TARGET_SUFFIX val)
include(UseDoxygen OPTIONAL)

add_subdirectory(parsing)
set(CMAKE_CXX_FLAGS "-Wall -std=gnu++11 ${CMAKE_CXX_FLAGS} -L/opt/ros/indigo/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system -lrosconsole -lmessage_store -lboost_thread")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${ParsePDDL_SOURCE_DIR})

set(libInst_SRCS
      SimpleEval.cpp
      FastEnvironment.cpp instantiation.cpp
      typecheck.cpp TypedAnalyser.cpp FuncAnalysis.cpp
      TIM.cpp TimSupport.cpp TypeStripWC.cpp
)

rosbuild_add_library(Inst STATIC ${libInst_SRCS})
target_link_libraries(Inst ParsePDDL Inst -L/opt/ros/indigo/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system -lmessage_store -lmongoclient -lboost_thread)
rosbuild_link_boost(Inst filesystem system)

set(validate_SRCS
     Action.cpp Proposition.cpp FuncExp.cpp typecheck.cpp main.cpp
     Validator.cpp RepairAdvice.cpp LaTeXSupport.cpp State.cpp Plan.cpp Ownership.cpp
     Environment.cpp Polynomial.cpp Utils.cpp
     TrajectoryConstraints.cpp RobustAnalyse.cpp random.cpp Events.cpp PrettyPrinter.cpp)
     
rosbuild_add_executable(validate ${validate_SRCS})

target_link_libraries(validate
        ParsePDDL
)

set(parser_SRCS
     parse.cpp
)

rosbuild_add_executable(parser ${parser_SRCS})

target_link_libraries(parser
        ParsePDDL
)

install(TARGETS validate parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

