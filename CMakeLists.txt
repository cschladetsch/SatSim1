cmake_minimum_required(VERSION 3.10)
project(SatelliteCommsProject VERSION 1.0)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define math constants for Windows
if(MSVC)
    add_definitions(-D_USE_MATH_DEFINES)
endif()

# Add the main library
add_library(SatelliteComms 
    SatelliteComms.cpp
)

# Add executable for the main application
add_executable(SatelliteApp main.cpp)
target_link_libraries(SatelliteApp PRIVATE SatelliteComms)

# Testing setup
enable_testing()

# Option to enable or disable tests
option(BUILD_TESTS "Build the tests" ON)
option(ENABLE_PERFORMANCE_TESTS "Enable performance tests" ON)

if(BUILD_TESTS)
    # Fetch and configure Google Test
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0  # Use a specific version
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    
    # Create a single test executable with all test files
    add_executable(SatelliteTests 
        tests/test.cpp
        tests/additional_tests.cpp
        tests/performance_tests.cpp
    )
    target_link_libraries(SatelliteTests PRIVATE 
        SatelliteComms
        gtest
        gtest_main
    )
    
    # Add compile definition to enable performance tests by default
    if(ENABLE_PERFORMANCE_TESTS)
        target_compile_definitions(SatelliteTests PRIVATE ENABLE_PERFORMANCE_TESTS)
    endif()
    
    # Add test to CTest with increased timeout if performance tests are enabled
    add_test(NAME SatelliteTests COMMAND SatelliteTests)
    if(ENABLE_PERFORMANCE_TESTS)
        set_tests_properties(SatelliteTests PROPERTIES TIMEOUT 300) # 5-minute timeout
    endif()
endif()

# Installation rules
install(TARGETS SatelliteComms SatelliteApp
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES SatelliteComms.h Vector3.h Quaternion.h DESTINATION include)
