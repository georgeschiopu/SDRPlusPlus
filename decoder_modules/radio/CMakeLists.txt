cmake_minimum_required(VERSION 3.13)
project(radio)

file(GLOB_RECURSE SRC "src/*.cpp")

include(${SDRPP_MODULE_CMAKE})

target_include_directories(radio PRIVATE "src/")
find_package(Threads REQUIRED)
target_link_libraries(radio PRIVATE Threads::Threads)

find_package(CURL)
if (CURL_FOUND)
    target_link_libraries(radio PRIVATE CURL::libcurl)
    target_compile_definitions(radio PRIVATE ADSB_HAVE_CURL)
endif()
