cmake_minimum_required(VERSION 3.10)
project(aduulm_launch)

find_package(aduulm_cmake_tools)
aduulm_init_ros2_package()

aduulm_install_ros2_python_code("${PROJECT_NAME}_py")

if(DEFINED BUILD_EXAMPLES)
    install(DIRECTORY
            examples
            DESTINATION share/${PROJECT_NAME}/
    )
endif()

# Finish the package creation and generate install() commands
aduulm_create_ros2_package()
setup_git_hooks(py_FORMATTERS "autopep8")
