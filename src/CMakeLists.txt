rock_library(advanced_navigation_anpp
    SOURCES Protocol.cpp
    HEADERS Protocol.hpp)

rock_executable(advanced_navigation_anpp_bin Main.cpp
    DEPS advanced_navigation_anpp)

