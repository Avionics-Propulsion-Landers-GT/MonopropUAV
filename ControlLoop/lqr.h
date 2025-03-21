#ifndef LQR_H
#define LQR_H

#include <Eigen/Dense>
#include <vector>

class LQR {
    public:
        LQR();
        ~LQR();
        
        Eigen::VectorXd setPoint;
        
        const Eigen::VectorXd& getState() const { return state; }
        const Eigen::MatrixXd& getK() const { return K;}

        void setA(const Eigen::MatrixXd& A);
        void setA(Eigen::MatrixXd&& A);
        void setB(const Eigen::MatrixXd& B);
        void setB(Eigen::MatrixXd&& B);
        void setQ(const Eigen::MatrixXd& Q);
        void setQ(Eigen::MatrixXd&& Q);
        void setR(const Eigen::MatrixXd& R);
        void setR(Eigen::MatrixXd&& R);
        void setK(const Eigen::MatrixXd& K);
        void setK(Eigen::MatrixXd&& K);
        void setState(const Eigen::VectorXd& state);
        void setState(Eigen::VectorXd&& state);

        void calculateK(); //calculate K matrix
        
    private:
        Eigen::VectorXd state;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd K;
};
#endif