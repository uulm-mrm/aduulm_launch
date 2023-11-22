cmake_minimum_required(VERSION 3.10)
project(aduulm_launch_lib)

# Get aduulm cmake macros for coloring, package_version extraction and find_aduulm_package()
find_package(aduulm_cmake_tools REQUIRED)
get_version_string_from_package_xml(.)
show_build_info("library")
setup_git_hooks(py_FORMATTERS "autopep8")

# install python code
# PYTHON_INSTALL_DIR is defined by aduulm_cmake_tools
install(DIRECTORY ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}_py/
        DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}_py)
