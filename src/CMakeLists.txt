rock_library(drivers-advanced_navigation_anpp
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(drivers-advanced_navigation_anpp_bin Main.cpp
    DEPS drivers-advanced_navigation_anpp)

