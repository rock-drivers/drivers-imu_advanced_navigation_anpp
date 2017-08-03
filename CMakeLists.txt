# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.1)
find_package(Rock)
rock_init(advanced_navigation_anpp 0.1)
rock_activate_cxx11()
rock_standard_layout()
