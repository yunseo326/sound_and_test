#include "../include/CalDegree.hpp"
#include "../include/AudioInput.hpp"

using namespace std; 
using namespace CalDegree;
 
namespace CalDegree {
    AudioResult getAudioAngle(const vector<double> &a, const vector<double> &b, const vector<double> &c, const vector<double> &d) {
    
        AudioResult cal_result;
        double frame1 = _gcc_phat(a, b);
        double frame2 = _gcc_phat(b, c);
        double frame3 = _gcc_phat(c, d);
        double frame4 = _gcc_phat(d, a);
     
        double dist_diff_1 = frame1 / SAMPLE_RATE * SPEED_SOUND / DISTANCE_MIC;
        double dist_diff_2 = frame2 / SAMPLE_RATE * SPEED_SOUND / DISTANCE_MIC;
        double dist_diff_3 = frame3 / SAMPLE_RATE * SPEED_SOUND / DISTANCE_MIC;
        double dist_diff_4 = frame4 / SAMPLE_RATE * SPEED_SOUND / DISTANCE_MIC;
        if (dist_diff_1 < -1) dist_diff_1 = -1;
        if (dist_diff_2 < -1) dist_diff_2 = -1;
        if (dist_diff_3 < -1) dist_diff_3 = -1;
        if (dist_diff_4 < -1) dist_diff_4 = -1;
        if (dist_diff_1 > 1) dist_diff_1 = 1;
        if (dist_diff_2 > 1) dist_diff_2 = 1;
        if (dist_diff_3 > 1) dist_diff_3 = 1;
        if (dist_diff_4 > 1) dist_diff_4 = 1;

        double alpha = std::acos(dist_diff_1) * 180.0 / M_PI;
        double beta  = std::acos(dist_diff_2) * 180.0 / M_PI;
        double gamma = std::acos(dist_diff_3) * 180.0 / M_PI;
        double omega = std::acos(dist_diff_4) * 180.0 / M_PI;
        
        int direction = _categorize_values(frame1,frame2,frame3,frame4);
        cal_result = _calculate_8_angles(alpha,beta,gamma,omega,direction);
        cal_result.direction = direction;
        return cal_result;
    }
}

// 마이크 간 시간 차이 및 최종 방향 계산
