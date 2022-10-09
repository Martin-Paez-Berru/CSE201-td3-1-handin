#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {

    double* array_new = new double[new_size];


    for (int i = 0; i < length; i++) {
        double temp = array[i];
        array_new[i] = temp;
    }

    for (int i = length; i <= new_size - 1; i++) {
        array_new[i] = 0;
    }

    delete array;

    return array_new;

}

double* shrink_array(double* array, int length, int new_size) {
    double* array_new = new double[new_size];

    for (int i = 0; i < new_size; i++) {
        double temp = array[i];
        array_new[i] = temp;
    }

    delete array;

    return array_new;

}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {

    if (current_size == max_size) {
        double* array_new = new double;
        array_new = extend_array(array, max_size, max_size + 5);
        array_new[current_size] = element;

        current_size = current_size + 1;
        max_size = max_size + 5;


        return array_new;
    }
    else {
        array[current_size] = element;

        current_size = current_size + 1;

        return array;
    }

}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {

    if (max_size - (current_size - 1) >= 5) {
        double* array_new = new double;
        array_new = shrink_array(array, current_size, current_size - 1);

        max_size = max_size - 5;
        current_size = current_size - 1;

        return array_new;

    }
    else {
        current_size = current_size - 1;

        return array;

    }

}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  // YOU CAN MODIFY THIS FUNCTION TO RECORD THE TELEMETRY

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;


  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;

    }
  }

  return hit_target;
}

void sort_td3(double *global_telemetry, const int global_telemetry_current_size) {
  for (int i = 0; i < global_telemetry_current_size; i ++) {
      for (int j = 2; j < global_telemetry_current_size; j++) {

          if (global_telemetry[(j-1)*3] > global_telemetry[i*3]) {

                double tempt = global_telemetry[i*3];
                global_telemetry[i*3] = global_telemetry[(j-1)*3];
                global_telemetry[(j-1)*3] = tempt;

                double tempx = global_telemetry[(i*3) + 1];
                global_telemetry[(i*3) + 1] = global_telemetry[(j-1)*3 + 1];
                global_telemetry[(j-1)*3 + 1] = tempx;

                double tempz = global_telemetry[(i*3) + 2];
                global_telemetry[(i*3) + 2] = global_telemetry[(j-1)*3 + 2];
                global_telemetry[(j-1)*3 + 2] = tempz;
          }
      }
  }
}



void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {

    for (int i = 0; i < tot_telemetries; i++) {
        for (int j = 0; j < telemetries_sizes[i]; j++) {
            global_telemetry = append_to_array(telemetries[i][j],
                                               global_telemetry,
                                               global_telemetry_current_size,
                                               global_telemetry_max_size);
        }
    }
 //   sort_td3(global_telemetry, global_telemetry_current_size);
}
