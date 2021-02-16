
#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>


double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

void Raycast(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& min,
             const Eigen::Vector2d& max, int& output_points_cnt, Eigen::Vector2d* output);

void Raycast(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& min,
             const Eigen::Vector2d& max, std::vector<Eigen::Vector2d>* output);

class RayCaster {
private:
  /* data */
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;
  Eigen::Vector2d direction_;
  Eigen::Vector2d min_;
  Eigen::Vector2d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  bool setInput(const Eigen::Vector2d& start,
                const Eigen::Vector2d& end /* , const Eigen::Vector2d& min,
                const Eigen::Vector2d& max */);

  bool step(Eigen::Vector2d& ray_pt);
};

#endif  // RAYCAST_H_