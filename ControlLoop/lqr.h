#ifndef LQR_H
#define LQR_H

#include "Vector.h"
#include "Matrix.h"

class LQR {
public:
    LQR();
    ~LQR();

    Vector setPoint;

    const Vector& getState() const { return state; }
    const Matrix& getK() const { return K; }

    // Lvalue setters
    void setA(const Matrix& A);
    void setA(Matrix&& A);
    void setB(const Matrix& B);
    void setB(Matrix&& B);
    void setQ(const Matrix& Q);
    void setQ(Matrix&& Q);
    void setR(const Matrix& R);
    void setR(Matrix&& R);
    void setK(const Matrix& K);
    void setK(Matrix&& K);
    void setState(const Vector& state);
    void setState(Vector&& state);

    void calculateK(double dt); // Calculate K matrix

private:
    Vector state;
    Matrix A;
    Matrix B;
    Matrix Q;
    Matrix R;
    Matrix K;
    double frobeniusNormDiff(const Matrix& A, const Matrix& B) const;
};

#endif // LQR_H
