
find_package(rqt_gui_cpp)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(PkgConfig)
pkg_check_modules(FC fontconfig REQUIRED IMPORTED_TARGET)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_cpp(MOC_SRCS
	topic_gui.h
)

qt5_wrap_ui(UI_SRCS
	topic_gui.ui
)

add_library(topic_gui
	${MOC_SRCS}
	${UI_SRCS}
	topic_gui.cpp
	plot_widget.cpp
	scrolling_buffer.cpp
	contrib/imgui/imgui.cpp
	contrib/imgui/imgui_draw.cpp
	contrib/imgui/imgui_demo.cpp
	contrib/imgui/imgui_tables.cpp
	contrib/imgui/imgui_widgets.cpp
	contrib/imgui/backends/imgui_impl_opengl3.cpp
	contrib/implot/implot.cpp
	contrib/implot/implot_items.cpp
)
add_dependencies(topic_gui
	${PROJECT_NAME}_generate_messages_cpp
)
target_include_directories(topic_gui PRIVATE
	contrib/imgui
	contrib/implot
)
target_link_libraries(topic_gui
	Qt5::Widgets
	PkgConfig::FC
)

install(TARGETS topic_gui
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
