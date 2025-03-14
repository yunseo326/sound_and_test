## Sound-source-localization-using-TDOA
This system utilizes a four-microphone array to detect the direction of a sound source. 
By combining the TDOA method with the GCC-PHAT, it precisely measures time differences between signals. Multithreading and C++ are used to optimize real-time data processing, ensuring high accuracy and low latency even in noisy environments.  



## execute the code 
#### 1. install the library in the "requirement.txt"


#### 2. execute the code (if you have error in cmake then change the CMakeLists.txt)
```cpp
mkdir build
cd build
cmake ..
make
./main
```
 
### Project File List
```cpp
device_check.cpp  : check the index of mics
Test_Degree.cpp   : test the process example except audio 
main.cpp          : main code
```

### Execution Flow of main code
```cpp
main  
↓  
process_audio  
↓  
gcc_phat, fft  
↓  
categorize_value  
↓  
calculate_8_angles  
↓  
select_final_direction  
```


### Module Descriptions
```cpp
final/src/_process_audio.cpp          : contain the core computation logic used in the program.  
final/src/_gcc_phat.cpp               : calculate the TDOA 
final/src/_fft.cpp                    : calculate the TDOA 
final/src/_categorize_values.cpp      : calculate the quadrant using TDOA
final/src/_calculate_8_angles.cpp     : calculate the 4angle using quadrant
final/src/_select_final_direction.cpp : return mean of two angle making minimum difference between them.
```