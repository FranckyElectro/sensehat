add_executable(main main.cpp)
target_link_libraries(main PUBLIC sense_hat_lib "pthread" "i2c")
