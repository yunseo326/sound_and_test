 
#include <iostream>
#include <portaudio.h>
int main() {
    Pa_Initialize(); // PortAudio 초기화

    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0) {
        std::cerr << "PortAudio 오류: " << Pa_GetErrorText(numDevices) << std::endl;
        return 1;
    }

    std::cout << "=== Available Audio Devices ===\n";
    for (int i = 0; i < numDevices; i++) {
        const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(i);
        std::cout << "ID: " << i << ", Name: " << deviceInfo->name
                  << ", Input Channels: " << deviceInfo->maxInputChannels
                  << ", Output Channels: " << deviceInfo->maxOutputChannels << std::endl;
    }

    Pa_Terminate(); // PortAudio 종료
    return 0;
}

