# Single test executable with all test files
add_executable(tests
        test_main.cpp
        Message.cpp
        MessageSet.cpp
        MessageDefinition.cpp
        MessageFieldIterator.cpp
        NoExceptions.cpp)

add_test(NAME tests COMMAND tests)

# Configure tests with coverage and no-exceptions support
target_compile_options(tests PRIVATE -O0 -g --coverage -Wall -Wextra -pedantic)
target_link_options(tests PRIVATE --coverage)
target_include_directories(tests PRIVATE ../include)
target_link_libraries(tests PRIVATE mav Threads::Threads)
