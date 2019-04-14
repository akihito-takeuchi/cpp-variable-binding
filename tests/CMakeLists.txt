macro(vbdAddTest test_name src_path)
  add_executable("${test_name}" "${src_path}")
  target_link_libraries("${test_name}" gtest gtest_main fmt vbd pthread)
  add_test(NAME "${test_name}"
    COMMAND $<TARGET_FILE:${test_name}>)
endmacro()

vbdAddTest("SimpleBind" "simplebind_test.cc")
vbdAddTest("CallbackTest" "callback_test.cc")
vbdAddTest("UpdateStopTest" "updatestop_test.cc")

set(QTDIR $ENV{QTDIR})
set(CMAKE_PREFIX_PATH ${QTDIR})
set(CMAKE_AUTOMOC on)
set(CMAKE_INCLUDE_CURRENT_DIR on)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

vbdAddTest("BindQt" "bindqt_test.cc")
qt5_use_modules(BindQt Widgets)
