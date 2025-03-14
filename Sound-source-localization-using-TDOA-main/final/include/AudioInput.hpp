// include/AudioInput.hpp
#ifndef AUDIOINPUT_HPP
#define AUDIOINPUT_HPP
 
#include <portaudio.h>
#include <vector>
#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER 44100
#define THRESHOLD 0.05
#define DEVICE_ID1 6
#define DEVICE_ID2 7
#define DEVICE_ID3 8
#define DEVICE_ID4 9

class AudioInput {
public:
    AudioInput(int deviceId, int sampleRate, int framesPerBuffer);
    ~AudioInput();

    bool openStream();
    bool startStream();
    bool stopStream();
    const std::vector<float>& getBuffer() const;

private:
    // PortAudio 콜백 함수는 static 함수여야 하므로, 클래스 내부에 정적으로 선언
    static int audioCallback(const void *inputBuffer, void *outputBuffer,
                             unsigned long framesPerBuffer,
                             const PaStreamCallbackTimeInfo *timeInfo,
                             PaStreamCallbackFlags statusFlags,
                             void *userData);

    PaStream *stream;
    int deviceId;
    int sampleRate;
    int framesPerBuffer;
    std::vector<float> buffer;  // 오디오 데이터 저장 버퍼
};

#endif // AUDIOINPUT_HPP
