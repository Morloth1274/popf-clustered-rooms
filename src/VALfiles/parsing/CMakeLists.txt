cmake_minimum_required(VERSION 2.4)

project(ParsePDDL)

find_program(BISON_EXECUTABLE bison)

if(NOT EXISTS ${BISON_EXECUTABLE})
	message("Could not find bison executable.")
endif(NOT EXISTS ${BISON_EXECUTABLE})


FIND_PROGRAM(FLEX_EXECUTABLE flex)

if(NOT EXISTS ${FLEX_EXECUTABLE})
        message("Could not find flex executable.")
endif(NOT EXISTS ${FLEX_EXECUTABLE})

add_custom_command(
       OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
       COMMAND
        ${FLEX_EXECUTABLE}
       ARGS
       -+  --outfile=${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc ${CMAKE_CURRENT_SOURCE_DIR}/pddl+.lex
       DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/pddl+.lex
)

add_custom_command(
       OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
       COMMAND
        ${BISON_EXECUTABLE}
       ARGS
        ${CMAKE_CURRENT_SOURCE_DIR}/pddl+.yacc -o ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
       DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/pddl+.yacc ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
       COMMAND
        ${CMAKE_CURRENT_SOURCE_DIR}/fixyywrap ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

set(libParsePDDL_SRCS
      pddl+.cpp ptree.cpp DebugWriteController.cpp
)

rosbuild_add_library(ParsePDDL STATIC ${libParsePDDL_SRCS})
    target_link_libraries(ParsePDDL
        -L/opt/ros/indigo/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system -lrosconsole -lmessage_store -lmongoclient -lboost_thread
    )

