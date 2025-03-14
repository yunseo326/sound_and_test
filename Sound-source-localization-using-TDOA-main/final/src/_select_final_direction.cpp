#include "../include/CalDegree.hpp"

namespace CalDegree {
    // 6개의 각도 중 가장 적은 오차를 보이는 두 각도를 선택하여 평균 계산
    std::pair<double, double> _select_final_direction(const std::vector<double>& angles) {
        double min_difference = std::numeric_limits<double>::infinity();
        std::pair<double, double> best_pair = {0, 0};
        
        for (size_t i = 0; i < angles.size(); i++) {
            for (size_t j = i + 1; j < angles.size(); j++) {
                double difference = std::abs(angles[i] - angles[j]);
                if (difference < min_difference) {
                    min_difference = difference;
                    best_pair = {angles[i], angles[j]};
                }
            }
        }
        return best_pair;
    }
}
 