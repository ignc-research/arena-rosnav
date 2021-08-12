#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>


using namespace std;

class UniformBspline{
private:
    // control points for B-spline with different dimensions.
    // Each column represents one single control point
    // The dimension is determined by row number
    // e.g. B-spline with N points in 3D space -> 3xN matrix
    // e.g. ctl_pts=[P0,P1,P2,P3,...Pn], Pi=[x;y;z]

    Eigen::MatrixXd control_points_;
    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    
    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio

public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info
    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);


    // compute position / derivative
    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    
    UniformBspline getDerivative();
    Eigen::MatrixXd getDerivativeControlPoints();




    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector2d> &point_set,
                                      const vector<Eigen::Vector2d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);


    /* check feasibility, adjust time */
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);
    
    /* reallocateTime, for optimize esdf */
    bool reallocateTime(bool show = false);


    /* for performance evaluation */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);



    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



#endif

