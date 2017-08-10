rock_library(imu_advanced_navigation_anpp
    SOURCES Protocol.cpp Driver.cpp
    HEADERS Protocol.hpp Driver.hpp Types.hpp Exceptions.hpp
    DEPS_PKGCONFIG iodrivers_base base-types gps_base)

rock_executable(imu_advanced_navigation_anpp_ctl Main.cpp
    DEPS imu_advanced_navigation_anpp)

