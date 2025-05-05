#ifndef QUATERNION_H
#define QUATERNION_H
#include "Vector.h"
#include "Matrix.h"

class Quaternion {
    protected:
        double* data;
    public:
        Quaternion();
        ~Quaternion();
        Quaternion(double w, double x, double y, double z);
        Quaternion(const Quaternion &other);
        Quaternion& operator=(const Quaternion &other);

        double& operator[](unsigned int index);
        const double& operator[](unsigned int index) const;
        Quaternion operator+(const Quaternion& other) const;
        Quaternion operator-(const Quaternion & other) const;
        Quaternion operator*(const Quaternion& other) const;
        Quaternion operator*(double scalar) const;
        Quaternion operator/(double scalar) const;
        Quaternion operator-() const;
        Quaternion conjugate() const;       
        double magnitude() const;
        Quaternion normalize() const;
        Quaternion inverse() const;
        Quaternion rotate(const Quaternion& q) const;
        // Quaternion fromEuler(const Vector& euler) const;
        // Vector toEuler() const;
        // Quaternion fromRotationMatrix(const Matrix& rot) const;
        Matrix toRotationMatrix() const;
        Matrix toEulerMatrix() const;

};
#endif