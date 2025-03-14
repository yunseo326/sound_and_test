#include "../include/CalDegree.hpp"
 
using namespace std;
using namespace CalDegree;
 
namespace CalDegree {
    AudioResult _calculate_8_angles(double alpha,double beta,double gamma,double omega, int direction) {

        double phi_alpha1;
        double phi_beta1;
        double phi_gamma1;
        double phi_omega1;
        
        AudioResult result;
        
        switch (direction) {
            case 0:
            cout << "direction error not calculate" << endl;
            result.angle_1 = 0;
            result.angle_2 = 0;
            result.angle_3 = 0;
            result.angle_4 = 0;
                   break;
            case 1:
            result.angle_1 = 90-alpha; 
            result.angle_2 = beta;
            result.angle_3 = gamma-90; 
            result.angle_4 = 180-omega;
                   break;
            case 2:
            result.angle_1 = 90+alpha;
            result.angle_2 = beta;
            result.angle_3 = 270-gamma;
            result.angle_4 = 180-omega;
                   break;
            case 3:
            result.angle_1 = 90+alpha;
            result.angle_2 = 360-beta;
            result.angle_3 = 270-gamma;
            result.angle_4 = 180+omega;
                   break;
            case 4:
            result.angle_1 = 450-alpha;
            result.angle_2 = 360-beta; 
            result.angle_3 = 270+gamma;
            result.angle_4 = 180+omega;
                   break;
        }
        return result;
      
    }
}
