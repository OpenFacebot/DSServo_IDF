# ds_servo/CMakeLists.txt
idf_component_register(SRCS "src/DSServo.cpp" 
                       INCLUDE_DIRS "include"
                       REQUIRES driver)