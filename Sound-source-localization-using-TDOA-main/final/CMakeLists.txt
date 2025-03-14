cmake_minimum_required(VERSION 3.10)
project(CalDegree_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
 
# 소스 파일 목록
set(SOURCES
    src/_calculate_8_angles.cpp
    src/_categorize_values.cpp
    src/_fft.cpp
    src/_gcc_phat.cpp
    src/_select_final_direction.cpp
    src/_process_audio.cpp
)

set(PORTAUDIO_INCLUDE_DIR /usr/include)
include_directories(${PORTAUDIO_INCLUDE_DIR})

# # FFTW3 라이브러리 찾기
# find_package(FFTW3 REQUIRED)\

# PortAudio 라이브러리 경로 수동 설정
set(PORTAUDIO_INCLUDE_DIR /usr/lib/python3/dist-packages/numpy/core/include)
set(PORTAUDIO_LIBRARY /usr/lib/x86_64-linux-gnu/libportaudio.so)

# PortAudio 경로 추가
include_directories(${PORTAUDIO_INCLUDE_DIR})
link_directories(${PORTAUDIO_LIBRARY})


# 실행 파일 생성
add_executable(caldegree_main Test_Degree.cpp ${SOURCES})
add_executable(main main.cpp ${SOURCES})
add_executable(device_check device_check.cpp)


# 헤더 파일 포함 디렉토리
include_directories(${PROJECT_SOURCE_DIR})

# FFTW 라이브러리 연결
target_link_libraries(caldegree_main PRIVATE fftw3)
target_link_libraries(main PRIVATE fftw3)


set(PORTAUDIO_LIBRARY /usr/lib/x86_64-linux-gnu/libportaudio.so)
target_link_libraries(main PRIVATE ${PORTAUDIO_LIBRARY})


find_package(Threads REQUIRED)
target_link_libraries(main PRIVATE Threads::Threads)

# 빌드 타입 설정
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message("Build Type: ${CMAKE_BUILD_TYPE}")
