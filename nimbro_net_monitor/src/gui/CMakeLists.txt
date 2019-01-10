
# Find Qt
find_package(Qt5Widgets REQUIRED)
find_package(Qt5WebKitWidgets REQUIRED)
find_package(Qt5DBus REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5WebKitWidgets_LIBRARIES} ${Qt5DBus_LIBRARIES} ${Qt5PrintSupport_LIBRARIES})

find_package(rqt_gui_cpp REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(contrib/qcustomplot)

qt5_wrap_cpp(MOC_SRCS
	monitor_display.h
	contrib/qcustomplot/qcustomplot.h
)

qt5_wrap_ui(UI_HDRS
	monitor_display.ui
)

add_library(nimbro_net_monitor_display
	${MOC_SRCS}
	${UI_HDRS}
	monitor_display.cpp
	contrib/qcustomplot/qcustomplot.cpp
)
add_dependencies(nimbro_net_monitor_display
	${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(nimbro_net_monitor_display
	${QT_LIBRARIES}
	${rqt_gui_cpp_LIBRARIES}
)

install(TARGETS nimbro_net_monitor_display
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
