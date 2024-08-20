set(LIBS "SenseHat.cpp;Hts221.cpp;Lps25h.cpp;I2c_Io.cpp")
set(INCLUDE "../include/")


##########################################
# Linking main.cpp with a shared library #
##########################################
add_library(sense_hat_lib SHARED ${LIBS})
target_include_directories(sense_hat_lib PUBLIC ${INCLUDE})
target_compile_features(sense_hat_lib PUBLIC cxx_std_11)
