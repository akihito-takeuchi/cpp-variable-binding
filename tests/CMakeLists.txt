macro(vbdAddTest test_name src_path)
  add_executable("${test_name}" "${src_path}")
  target_link_libraries("${test_name}" gtest gtest_main fmt vbd pthread)
  add_test(NAME "${test_name}"
    COMMAND $<TARGET_FILE:${test_name}>)
endmacro()

vbdAddTest("SimpleBind" "simplebind_test.cc")
vbdAddTest("CallbackTest" "callback_test.cc")
vbdAddTest("UpdateStopTest" "updatestop_test.cc")
