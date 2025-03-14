#include "../include/CalDegree.hpp"
#include <fftw3.h>
 
namespace CalDegree {
    void _fft(CVector &signal, bool inverse) {
        int N = signal.size();
        fftw_plan plan;
    
        if (inverse) {
            // 역 푸리에 변환 (IFFT)
            plan = fftw_plan_dft_1d(N, reinterpret_cast<fftw_complex*>(signal.data()),
                                    reinterpret_cast<fftw_complex*>(signal.data()), FFTW_BACKWARD, FFTW_ESTIMATE);
        } else {
            // 푸리에 변환 (FFT)
            plan = fftw_plan_dft_1d(N, reinterpret_cast<fftw_complex*>(signal.data()),
                                    reinterpret_cast<fftw_complex*>(signal.data()), FFTW_FORWARD, FFTW_ESTIMATE);
        }
    
        fftw_execute(plan);  // 변환 수행
        fftw_destroy_plan(plan);
    }
}

