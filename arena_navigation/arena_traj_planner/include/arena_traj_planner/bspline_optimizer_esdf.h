#ifndef _BSPLINE_OPTIMIZER_ESDF_H_
#define _BSPLINE_OPTIMIZER_ESDF_H_



#include <Eigen/Eigen>
#include <arena_mapping/mapping.h>
#include <ros/ros.h>
class BsplineOptimizerESDF {

public:
    static const int SMOOTHNESS;
    static const int DISTANCE;
    static const int FEASIBILITY;
    static const int ENDPOINT;
    static const int GUIDE;
    static const int WAYPOINTS;

    static const int GUIDE_PHASE;
    static const int NORMAL_PHASE;

    BsplineOptimizerESDF() {}
    ~BsplineOptimizerESDF() {}

    /* main API */
    void            setEnvironment(const GridMap::Ptr& env);
    void            setParam(ros::NodeHandle& nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

    // required inputs
    void setControlPoints(const Eigen::MatrixXd& points);
    void setBsplineInterval(const double& ts);
    void setCostFunction(const int& cost_function);
    void setTerminateCond(const int& max_num_id, const int& max_time_id);


    // optional inputs
    void setGuidePath(const std::vector<Eigen::Vector2d>& guide_pt);
    void setWaypoints(const std::vector<Eigen::Vector2d>& waypts,
                        const std::vector<int>&             waypt_idx);  // N-2 constraints at most


    void optimize();

    Eigen::MatrixXd         getControlPoints();
    std::vector<Eigen::Vector2d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);


private:
    GridMap::Ptr grid_map_;
    // main input
    Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double          bspline_interval_;   // B-spline knot span
    Eigen::Vector2d end_pt_;             // end of the trajectory
    int             dim_;                // dimension of the B-spline


    std::vector<Eigen::Vector2d> guide_pts_;  // geometric guiding path points, N-6
    std::vector<Eigen::Vector2d> waypoints_;  // waypts constraints
    std::vector<int>             waypt_idx_;  // waypts constraints index

    int    max_num_id_, max_time_id_;    // stopping criteria
    int    cost_function_;               // used to determine objective function
    bool   dynamic_;                     // moving obstacles ?
    double start_time_;                  // global time for moving obstacles

    /* optimization parameters */
    int    order_;                  // bspline degree
    double lambda1_;                // jerk smoothness weight
    double lambda2_;                // distance weight
    double lambda3_;                // feasibility weight
    double lambda4_;                // end point weight
    double lambda5_;                // guide cost weight
    double lambda6_;                // visibility cost weight
    double lambda7_;                // waypoints cost weight
    double lambda8_;                // acc smoothness


    double dist0_;                  // safe distance
    double max_vel_, max_acc_;      // dynamic limits
    double visib_min_;              // threshold of visibility
    double wnl_;                    //
    double dlmin_;                  //

    int    algorithm1_;             // optimization algorithms for quadratic cost
    int    algorithm2_;             // optimization algorithms for general cost
    int    max_iteration_num_[4];   // stopping criteria that can be used
    double max_iteration_time_[4];  // stopping criteria that can be used

    /* intermediate variables */
    /* buffer for gradient of cost function, to avoid repeated allocation and
    * release of memory */
    std::vector<Eigen::Vector2d> g_q_;
    std::vector<Eigen::Vector2d> g_smoothness_;
    std::vector<Eigen::Vector2d> g_distance_;
    std::vector<Eigen::Vector2d> g_feasibility_;
    std::vector<Eigen::Vector2d> g_endpoint_;
    std::vector<Eigen::Vector2d> g_guide_;
    std::vector<Eigen::Vector2d> g_waypoints_;

    int                 variable_num_;   // optimization variables
    int                 iter_num_;       // iteration of the solver
    std::vector<double> best_variable_;  //
    double              min_cost_;       //

    std::vector<Eigen::Vector2d> block_pts_;  // blocking points to compute visibility


    /* cost function */
    /* calculate each part of cost function with control points q as input */
    static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
    void          combineCost(const std::vector<double>& x, std::vector<double>& grad, double& cost);

    // q contains all control points
    void calcSmoothnessCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                          std::vector<Eigen::Vector2d>& gradient);
    void calcDistanceCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                        std::vector<Eigen::Vector2d>& gradient);
    void calcFeasibilityCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                           std::vector<Eigen::Vector2d>& gradient);
    void calcEndpointCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                        std::vector<Eigen::Vector2d>& gradient);
    void calcGuideCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
    void calcVisibilityCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                          std::vector<Eigen::Vector2d>& gradient);
    void calcWaypointsCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                         std::vector<Eigen::Vector2d>& gradient);
    void calcViewCost(const std::vector<Eigen::Vector2d>& q, double& cost, std::vector<Eigen::Vector2d>& gradient);
    bool isQuadratic();



/* for benckmark evaluation only */
public:
    std::vector<double> vec_cost_;
    std::vector<double> vec_time_;
    ros::Time      time_start_;

    void getCostCurve(std::vector<double>& cost, std::vector<double>& time) {
        cost = vec_cost_;
        time = vec_time_;
    }


    typedef std::unique_ptr<BsplineOptimizerESDF> Ptr;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};











#endif