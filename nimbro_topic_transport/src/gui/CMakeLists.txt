
# Decide on a Qt version
find_package(rqt_gui_cpp)
message(STATUS "RQT uses Qt version ${qt_gui_cpp_USE_QT_MAJOR_VERSION}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
	set(USE_QT5 true)
else()
	set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
	find_package(Qt5Widgets REQUIRED)
	find_package(Qt5WebKitWidgets REQUIRED)
	find_package(Qt5DBus REQUIRED)
	find_package(Qt5PrintSupport REQUIRED)
	set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5WebKitWidgets_LIBRARIES} ${Qt5DBus_LIBRARIES} ${Qt5PrintSupport_LIBRARIES})
else()
	find_package(Qt4 COMPONENTS QtCore QtGui QtWebKit QtDBus REQUIRED)
	include(${QT_USE_FILE})
endif()

macro(wrap_cpp)
	if(USE_QT5)
		qt5_wrap_cpp(${ARGV})
	else()
		qt4_wrap_cpp(${ARGV})
	endif()
endmacro()

wrap_cpp(MOC_SRCS
	topic_gui.h
	dot_widget.h
)

wrap_cpp(BAND_MOC_SRCS
	bandwidth_gui.h
	contrib/qcustomplot/qcustomplot.h
)

add_library(topic_gui
	${MOC_SRCS}
	topic_gui.cpp
	dot_widget.cpp
)
add_dependencies(topic_gui
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(topic_gui
	${QT_LIBRARIES}
)

add_library(bandwidth_gui
	${BAND_MOC_SRCS}
	bandwidth_gui.cpp
	contrib/qcustomplot/qcustomplot.cpp
)
add_dependencies(bandwidth_gui
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(bandwidth_gui
	${QT_LIBRARIES}
	yaml-cpp
)

install(TARGETS topic_gui bandwidth_gui
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
