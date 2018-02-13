#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

include_directories(${PROJECT_SOURCE_DIR}/include)
ADD_EXECUTABLE( UnitTest.run
    gtest_main.cpp
    TVideoSource.cpp
    TImgProc.cpp
    TKeyFrame.cpp
    TATANCamera.cpp
    THomography.cpp
    TMathUtility.cpp
    TJsonConfig.cpp
    ${SRC_LIST_PTAM} )
TARGET_LINK_LIBRARIES(UnitTest.run gtest pthread)
