target_sources(${PROJECT_NAME}
  PRIVATE     
    "${CMAKE_CURRENT_SOURCE_DIR}/BallRegionCompute.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/GenerateRemesh.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/PlyLoad.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/trackball.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/GLM/glm.cpp"
#    "${CMAKE_CURRENT_SOURCE_DIR}/FileProcess/CBrowseDir.cpp"
#    "${CMAKE_CURRENT_SOURCE_DIR}/FileProcess/LoadFileDlg.cpp"
#    "${CMAKE_CURRENT_SOURCE_DIR}/FileProcess/CStatDir.hpp"
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/BallRegionCompute.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/GeneralSimplification.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/LoadPointCloud.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Method_AIVS_OM.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Method_AIVS_Remesh.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Method_CGAL_Advancing.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/normalCompute.hpp"    
    "${CMAKE_CURRENT_SOURCE_DIR}/pointProcessPipeline.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/VoronoiDiagram.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/PlyLoad.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/trackball.h"    
#    "${CMAKE_CURRENT_SOURCE_DIR}/View.h"
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  "${CMAKE_CURRENT_SOURCE_DIR}/"
  "${CMAKE_CURRENT_SOURCE_DIR}/GLM"
  "${CMAKE_CURRENT_SOURCE_DIR}/FileProcess"
  ${CGAL_INCLUDE_DIRS} )


  target_link_libraries(${PROJECT_NAME} PUBLIC 
    optimized ${PCL_LIBRARIES}  
    ${CGAL_LIBRARIES}
  )

  #target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)