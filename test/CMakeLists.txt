set(LIBS "../sensehat/SenseHat.cpp;../sensehat/Hts221.cpp;../sensehat/Lps25h.cpp;../sensehat/I2c_Io.cpp")
add_executable(unit_tests
    unit_tests.cpp
    ${LIBS})
target_link_libraries(unit_tests PUBLIC sense_hat_lib "pthread" "i2c")
target_include_directories(unit_tests PRIVATE ../include)
add_test(NAME connectivity COMMAND unit_tests 1)
