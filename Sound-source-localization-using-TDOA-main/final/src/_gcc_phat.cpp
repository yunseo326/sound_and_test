#include "../include/CalDegree.hpp"
using namespace std;
using namespace CalDegree;

namespace CalDegree {
    // GCC-PHAT 계산 함수
    double _gcc_phat(const vector<double> &x, const vector<double> &y) {
        int N = x.size();

        // 입력 신호를 복소수로 변환 (정상적인 FFT를 위해)
        CVector X(N), Y(N);
        for (int i = 0; i < N; ++i) {
            X[i] = Complex(x[i], 0.0);
            Y[i] = Complex(y[i], 0.0);
        }

        // FFT 수행
        _fft(X);
        _fft(Y);

        // Cross Power Spectrum 계산 (PHAT 적용)
        CVector X_conj_Y(N);
        for (int i = 0; i < N; ++i) {
            X_conj_Y[i] = conj(X[i]) * Y[i] / abs(X[i]);  // PHAT: 위상 정보만 사용
        }

        // 역 FFT 수행하여 시간 도메인으로 변환
        _fft(X_conj_Y, true);

        // 가장 큰 값을 찾음 (시간 지연)
        double max_corr = 0.0;
        int max_index = 0;
        for (int i = 0; i < N; ++i) {
            if (abs(X_conj_Y[i]) > max_corr) {
                max_corr = abs(X_conj_Y[i]);
                max_index = i;
            }
        }

        // 시간 지연 (초 단위로 반환)
        double delay_samples = max_index;
        if (delay_samples > N / 2) {
            delay_samples -= N;  // 시간 지연이 음수일 수 있기 때문
        }
        return delay_samples;
    }
}

 