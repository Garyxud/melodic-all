
include_directories( ./ ../  ../../include  ../../common ${Qt5WebSockets_INCLUDE_DIRS})

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_WEBSOCKETS)

find_package(Qt5 REQUIRED COMPONENTS WebSockets )

#QT5_WRAP_UI ( UI_SRC  ../common/selectlistdialog.ui  )


SET( SRC
    datastreamserver.cpp
    ../../include/PlotJuggler/datastreamer_base.h
    )

add_library(DataStreamWebSocket SHARED ${SRC} ${UI_SRC}  )
target_link_libraries(DataStreamWebSocket  ${Qt5Widgets_LIBRARIES} Qt5::WebSockets Qt5::Xml)


