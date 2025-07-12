add_executable(tests
        test_main.cpp)

# Add separate no-exceptions test executable
add_executable(test_no_exceptions
        test_main.cpp
        NoExceptions.cpp)

add_test(NAME tests COMMAND tests)
add_test(NAME test_no_exceptions COMMAND test_no_exceptions)

target_compile_options(tests PRIVATE -O0 -g --coverage -Wall -Wextra -pedantic)
target_link_options(tests PRIVATE --coverage)
target_include_directories(tests PRIVATE ../include)
target_link_libraries(tests PRIVATE mav Threads::Threads)

# Configure no-exceptions test
target_compile_options(test_no_exceptions PRIVATE -fno-exceptions -Wall -Wextra -pedantic)
target_include_directories(test_no_exceptions PRIVATE ../include)
target_link_libraries(test_no_exceptions PRIVATE mav Threads::Threads)
