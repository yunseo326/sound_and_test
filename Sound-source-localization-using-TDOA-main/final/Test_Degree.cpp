#include <iostream>
#include <vector>
#include <random>
#include "include/CalDegree.hpp"

using namespace std;
using namespace CalDegree;

int main() {
    const int SIZE = 100;
    vector<double> mic1(SIZE), mic2(SIZE), mic3(SIZE), mic4(SIZE);

    // 랜덤 생성기 초기화
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(-10.0, 10.0);

    // 각 벡터를 -10 ~ 10 사이의 랜덤 값으로 채우기
    for (int i = 0; i < SIZE; ++i) {
        mic1[i] = dis(gen);
        mic2[i] = dis(gen);
        mic3[i] = dis(gen);
        mic4[i] = dis(gen);
    }

    // 각 벡터의 중간(인덱스 50)에 100으로 튀는 값을 삽입
    int mid = SIZE / 2;  // 100개의 경우 mid는 50 (0~99에서 50번째 요소)
    mic1[20] = 100.0;
    mic2[46] = 100.0;
    mic3[46] = 100.0; 
    mic4[71] = 100.0;

    int sample_rate = 44100; // 샘플링 레이트 설정

    // 방향 각도 계산 (process_audio 함수가 4개의 벡터와 sample_rate를 인자로 받음)
    AudioResult result = getAudioAngle(mic1, mic2, mic3, mic4);

    // 결과 출력
    cout << "계산된 최종 방향: " << result.direction << endl;
    cout << "Angle 1: " << result.angle_1 << " 도" << endl;
    cout << "Angle 2: " << result.angle_2 << " 도" << endl;
    cout << "Angle 3: " << result.angle_3 << " 도" << endl;
    cout << "Angle 4: " << result.angle_4 << " 도" << endl;
    cout << "최종 방향: " << (result.angle_1 + result.angle_2 + result.angle_3 + result.angle_4) / 4 << " 도" << endl;

    return 0;
}
