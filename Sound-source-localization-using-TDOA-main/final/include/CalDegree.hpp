#ifndef CALDEGREE_HPP
#define CALDEGREE_HPP
 
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>
#include <complex>

using namespace std;

typedef complex<double> Complex;
typedef vector<Complex> CVector;


namespace CalDegree {
    struct AudioResult{
        double angle_1;
        double angle_2;
        double angle_3;
        double angle_4;
        int direction;
    };

    const double DISTANCE_MIC = 0.2; // 마이크 간 거리
    const double SPEED_SOUND = 343.0;
    // 마이크 간 시간 차이 계산
    void _fft(CVector &signal, bool inverse = false);
    double _gcc_phat(const vector<double> &x, const vector<double> &y);
    
    // 각도 계산을 위한 도구
    int _categorize_values(double value, double value2, double value3, double value4);
    AudioResult _calculate_8_angles(double alpha,double beta,double gamma,double omega, int direction);
    std::pair<double, double> _select_final_direction(const std::vector<double>& angles);
    
    //각도 계산
    AudioResult getAudioAngle(const vector<double> &a, const vector<double> &b, const vector<double> &c, const vector<double> &d);
}
#endif // CALDEGREE_HPP