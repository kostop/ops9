#include "smooth_speed_filter.h"


double smooth_speed_realtime(double current_speed, double *history_speed, int size) {
    double sum = 0;
    for (int i = 0; i < size; i++) {
        sum += history_speed[i];
    }
    double average_speed = sum / size;
    for (int i = 0; i < size - 1; i++) {
        history_speed[i] = history_speed[i + 1];
    }
    history_speed[size - 1] = current_speed;
    sum = 0;
    for (int i = 0; i < size; i++) {
        sum += history_speed[i];
    }
    return sum / size;
}


//void smooth_speed(double *speed, int size, int window_size) {
//    double* temp = (double*)malloc(size * sizeof(double)); // 用于存储滤波后的速度
//    int half_window_size = window_size / 2;
//    for (int i = 0; i < size; i++) {
//        double sum = 0;
//        int count = 0;
//        for (int j = -half_window_size; j <= half_window_size; j++) {
//            int idx = i + j;
//            if (idx >= 0 && idx < size) {
//                sum += speed[idx];
//                count++;
//            }
//        }
//        temp[i] = sum / count;
//    }
//    // 将滤波后的速度复制回原速度
//    for (int i = 0; i < size; i++) {
//        speed[i] = temp[i];
//    }
//    free(temp);
//}


