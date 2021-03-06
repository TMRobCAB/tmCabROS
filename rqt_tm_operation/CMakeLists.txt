cmake_minimum_required(VERSION 2.8.3)

project(rqt_tm_operation)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_tm_operation_SRCS
  src/rqt_tm_operation/tm_operation.cpp
)

set(rqt_tm_operation_HDRS
  include/rqt_tm_operation/tm_operation.h
)

set(rqt_tm_operation_UIS
  src/rqt_tm_operation/tm_operation.ui
)

set(rqt_tm_operation_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rqt_tm_operation_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(rqt_tm_operation_MOCS ${rqt_tm_operation_HDRS})
qt4_wrap_ui(rqt_tm_operation_UIS_H ${rqt_tm_operation_UIS})

include_directories(${rqt_tm_operation_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_tm_operation_SRCS} ${rqt_tm_operation_MOCS} ${rqt_tm_operation_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_tm_operation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
