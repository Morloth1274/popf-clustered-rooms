PROJECT(popf)

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}
                          ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules)


find_package(Clp)
find_package(CoinUtils)
find_package(Cbc)
find_package(Cgl)
find_package(OSI)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(catkin REQUIRED COMPONENTS tf)

set(CMAKE_CXX_FLAGS "-Wall -std=gnu++11 ${CMAKE_CXX_FLAGS}")

if(STATIC_POPF)
set(CMAKE_CXX_LINK_FLAGS "-static -std=gnu++11 ${CMAKE_CXX_LINK_FLAGS}")
endif(STATIC_POPF)

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${val_BINARY_DIR} ${CLP_INCLUDES}/coin ${CBC_INCLUDES}/coin ${CPLEX_INCLUDES} ${CONCERT_INCLUDES} ${CATKIN_INCLUDES})

set(popf_build_srcs
    totalordertransformer.cpp
    partialordertransformer.cpp
    FFSolver.cpp
    globals.cpp
    lpscheduler.cpp
    RPGBuilder.cpp
    RPGBuilderAnalysis.cpp
    RPGBuilderEvaluation.cpp
    RPGBuilderNumerics.cpp
    temporalanalysis.cpp
    minimalstate.cpp
    temporalconstraints.cpp
    numericanalysis.cpp
    solver.cpp)

rosbuild_add_library(popfCommon STATIC ${popf_build_srcs})
target_link_libraries(popfCommon ParsePDDL Inst -L/opt/ros/indigo/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system -lrosconsole -lmongoclient -lboost_thread -lboost_filesystem -lssl -lcrypto)
    install(TARGETS popfCommon
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

#if((NOT DISABLE_CLP) AND CLP_INCLUDES)
if (CLP_INCLUDES AND CBC_INCLUDES)
    rosbuild_add_executable(popf popfMain.cpp solver-clp.cpp)
    
    target_link_libraries(popf
        popfCommon
        ${CBC_SOLVER_LIBRARY} ${CBC_LIBRARY} ${CGL_LIBRARIES} ${OSICLP_LIBRARIES} ${OSI_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES} ${CATKIN_LIBRARIES} -L/opt/ros/indigo/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system -lrosconsole -lmessage_store -pthread -lmongoclient -lboost_filesystem -lboost_program_options -lboost_system -lboost_thread -lssl -lcrypto
    )
#rosbuild_add_boost_directories(../libs/glm /usr/include/freetype2)
rosbuild_link_boost(popf filesystem system)
    install(TARGETS popf
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
                
#endif((NOT DISABLE_CLP) AND CLP_INCLUDES)
endif(CLP_INCLUDES AND CBC_INCLUDES)

if(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)
    rosbuild_add_executable(popf-cplex popfMain.cpp solver-cpx.cpp)
    
    target_link_libraries(popf-cplex    
        popfCommon
        ${ILOCPLEX} ${CPLEX} ${CONCERT} pthread
    )
    install(TARGETS popf-cplex
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    

endif(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)


