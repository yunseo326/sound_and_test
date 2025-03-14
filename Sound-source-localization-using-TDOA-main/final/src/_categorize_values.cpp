// 4분면 정하기 
#include <iostream>
namespace CalDegree {
    int _categorize_values(double value, double value2, double value3, double value4) {
        std:: cout << value - value3 << " " << value2 - value4 << " " << std:: endl;
        if (value - value3 >= 0) {
            if (value2 - value4 >= 0){
                return 1;
            } 
            else if (value2 - value4 <= 0){
                return 2;
            } 
            else {
                return 0;
            }
        } 
        else if (value - value3 <= 0) {
            if (value2 - value4 >= 0) {
                return 4;
            } 
            else if (value2 - value3 <= 0){
                return 3;
            } 
            else {
                return 0;
            }
        }
        return 0;
    }
}
    

