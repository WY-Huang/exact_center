# CMake generated Testfile for 
# Source directory: /home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center
# Build directory: /home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/copyright.xunit.xml" "--package-name" "test_laser_center" "--output-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/ament_copyright/copyright.txt" "--command" "/opt/ros/galactic/bin/ament_copyright" "--xunit-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_copyright.cmake;46;ament_add_test;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;18;ament_copyright;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;57;ament_package;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/cppcheck.xunit.xml" "--package-name" "test_laser_center" "--output-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/galactic/bin/ament_cppcheck" "--xunit-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/cppcheck.xunit.xml" "--include_dirs" "/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;61;ament_add_test;/opt/ros/galactic/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;83;ament_cppcheck;/opt/ros/galactic/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;57;ament_package;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;0;")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/cpplint.xunit.xml" "--package-name" "test_laser_center" "--output-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/ament_cpplint/cpplint.txt" "--command" "/opt/ros/galactic/bin/ament_cpplint" "--xunit-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;68;ament_add_test;/opt/ros/galactic/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;35;ament_cpplint;/opt/ros/galactic/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;57;ament_package;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/lint_cmake.xunit.xml" "--package-name" "test_laser_center" "--output-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/galactic/bin/ament_lint_cmake" "--xunit-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;57;ament_package;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/uncrustify.xunit.xml" "--package-name" "test_laser_center" "--output-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/galactic/bin/ament_uncrustify" "--xunit-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;65;ament_add_test;/opt/ros/galactic/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;34;ament_uncrustify;/opt/ros/galactic/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;57;ament_package;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/xmllint.xunit.xml" "--package-name" "test_laser_center" "--output-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/ament_xmllint/xmllint.txt" "--command" "/opt/ros/galactic/bin/ament_xmllint" "--xunit-file" "/home/wanyel/vs_code/exact_center/ros2_test_center/build/test_laser_center/test_results/test_laser_center/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;57;ament_package;/home/wanyel/vs_code/exact_center/ros2_test_center/src/test_laser_center/CMakeLists.txt;0;")
