# ds_servo/CMakeLists.txt
idf_component_register(SRCS "DSServo.cpp" 
                       INCLUDE_DIRS "include"
                       REQUIRES driver)