rock_library(advanced_navigation_anpp
    SOURCES Protocol.cpp Driver.cpp
    HEADERS Protocol.hpp Driver.hpp Types.hpp Exceptions.hpp
    DEPS_PKGCONFIG iodrivers_base base-types gps_base)

rock_executable(advanced_navigation_anpp_bin Main.cpp
    DEPS advanced_navigation_anpp)

