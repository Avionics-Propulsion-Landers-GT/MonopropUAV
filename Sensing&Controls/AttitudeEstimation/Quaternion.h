//
// Created by Sarang Suman on 3/11/25.
//

#ifndef QUATERNION_H
#define QUATERNION_H



class Quaternion {
public:
    double w, x, y, z;

    Quaternion(double w, double x, double y, double z);

    Quaternion conjugate() const;
    Quaternion add(const Quaternion& q) const;
    Quaternion multiply(const Quaternion& q) const;
    double norm() const;
    void toRotationMatrix(double matrix[3][3]) const;
};



#endif //QUATERNION_H
