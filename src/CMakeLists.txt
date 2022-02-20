cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)

include_directories("C:/libs/SFML-2.5.1/include" "C:/libs/boost/include")
link_directories("C:/libs/SFML-2.5.1/lib/" "C:/libs/boost/lib")

set(DBG_LIBS sfml-main-d.lib sfml-graphics-d.lib sfml-window-d.lib sfml-system-d.lib)
set(RLS_LIBS sfml-main.lib sfml-graphics.lib sfml-window.lib sfml-system.lib)

project(polyphys)
add_executable(polyphys main.cpp collision.cpp manifold.cpp world.cpp)

target_link_libraries(polyphys debug ${DBG_LIBS} optimized ${RLS_LIBS})

