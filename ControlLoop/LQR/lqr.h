#ifndef LQR_H
#define LQR_H

#include "../CustomLinear/Vector.h"
#include "../CustomLinear/Matrix.h"

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
    void setX(Matrix&& X);
    void setState(const Vector& state);
    void setState(Vector&& state);

    double calculateK(double dt, double fnX); // Calculate K matrix

    double frobeniusNormDiff(const Matrix& A, const Matrix& B) const;
    double frobeniusNorm(const Matrix& A) const;

private:
    Vector state;
    Matrix A;
    Matrix B;
    Matrix Q;
    Matrix R;
    Matrix K;
};

#endif // LQR_H
