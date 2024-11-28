#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

// Hier kannst du deine Header-Dateien einfügen
#include <eigen3/Eigen/Dense> // hat to do this, for me it does not find #include <Eigen/Dense> only witheigen3/ infront
#include <eigen3/Eigen/Geometry>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;


// Hier kannst du deine Funktionen und Klassen deklarieren
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return m;
    
}

double sign(const double* num) {
    return (*num >= 0) - (*num < 0);
}

Vector6d limitVector(const Vector6d& input, const Vector6d& max_vector) {
    Vector6d result = input;
    for(int i = 0; i < 6; i++) {
        if(std::abs(result(i)) > max_vector(i)) {
            result(i) = sign(&result(i)) * max_vector(i);
        }
    }
    return result;
}

Vector3d limitErrorVector(const Vector3d& input, const Vector3d& max_vector) {
    Vector3d result = input;
    for(int i = 0; i < 3; i++) {
        if(std::abs(result(i)) > max_vector(i)) {
            result(i) = sign(&result(i)) * max_vector(i);
        }
    }
    return result;
}

Eigen::Vector3d limitVector3D(const Eigen::Vector3d& input, const Eigen::Vector3d& max_vector) {
    Eigen::Vector3d result = input;
    for(int i = 0; i < 3; i++) {
        if(std::abs(result(i)) > max_vector(i)) {
            result(i) = sign(&result(i)) * max_vector(i);
        }
    }
    return result;
}

Eigen::Vector3d skew_Matrix_inverse(const Eigen::Matrix3d& m) {
    Eigen::Vector3d v;
    v << m(2, 1), m(0, 2), m(1, 0);
    return v;
}

Eigen::Vector3d pwm_ranger(const Eigen::Vector3d& pwm, float pwm_min, float pwm_max) {
    Eigen::Vector3d result = pwm;
    for (int i = 0; i < 3; i++) {
        if (result(i) < 0) {
            result(i) = -result(i);
        }
    }

    
    for (int i = 0; i < 3; i++) {
        if (result(i) < pwm_min) {
            result(i) = pwm_min;
        } else if (result(i) > pwm_max) {
            result(i) = pwm_max;
        }
    }

        return result;
    
}

Vector6d vec_get_nearest_rotation(const Vector6d& des_angles, const Vector6d& angles_now) {
    double distance;
    Vector6d nearest_rot_to_des_angle;
    nearest_rot_to_des_angle.setZero();

    for(int i = 0; i < 6; i++) {

        distance = std::fmod(des_angles(i) - angles_now(i), 2 * M_PI);

        if (distance < -M_PI) {
            distance += 2 * M_PI;
        } else if (distance > M_PI) {
            distance -= 2 * M_PI;
        }

        nearest_rot_to_des_angle(i) = angles_now(i) + distance;
    }
    


    return nearest_rot_to_des_angle;
}



#endif // HELPERFUNCTIONS_H