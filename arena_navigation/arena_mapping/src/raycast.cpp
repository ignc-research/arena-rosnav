
#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <arena_mapping/raycast.h>


int signum(int x) {
    /*if x=0,return 0; if x<0, return -1; if x>0, return 1*/
    return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

void Raycast(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& min,
             const Eigen::Vector2d& max, int& output_points_cnt, Eigen::Vector2d* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  Eigen::Vector2d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);


  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
 

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;


  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y()) {
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;


      output_points_cnt++;
      dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)));

      if (dist > maxDist) return;

      /*            if (output_points_cnt > 1500) {
                      std::cerr << "Error, too many racyast voxels." <<
         std::endl;
                      throw std::out_of_range("Too many raycast voxels");
                  }*/
    }

    if (x == endX && y == endY) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY) {
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
    } else {
      
        y += stepY;
        tMaxY += tDeltaY;
    } 
    
  }
}



void Raycast(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& min,
             const Eigen::Vector2d& max, std::vector<Eigen::Vector2d>* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());

  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());

  Eigen::Vector2d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;


  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);


  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);


  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;


  output->clear();

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 ) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() ) {
      output->push_back(Eigen::Vector2d(x, y));

      dist = (Eigen::Vector2d(x, y) - start).squaredNorm();

      if (dist > maxDist) return;

      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    if (x == endX && y == endY) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY) {
      
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
    } else {
        y += stepY;
        tMaxY += tDeltaY;
    }
  }
}



bool RayCaster::setInput(const Eigen::Vector2d& start,
                         const Eigen::Vector2d& end /* , const Eigen::Vector2d& min,
                         const Eigen::Vector2d& max */) {
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());

  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  // Break out direction vector.
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;

  // Direction to increment x,y,z when stepping.
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);


  // The change in t when taking a step (always positive).
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;

  dist_ = 0;

  step_num_ = 0;

  // Avoids an infinite loop.
  if (stepX_ == 0 && stepY_ == 0)
    return false;
  else
    return true;
}


bool RayCaster::step(Eigen::Vector2d& ray_pt) {

  ray_pt = Eigen::Vector2d(x_, y_);

  // step_num_++;

  // dist_ = (Eigen::Vector2d(x_, y_, z_) - start_).squaredNorm();

  if (x_ == endX_ && y_ == endY_) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  if (tMaxX_ < tMaxY_) {
      x_ += stepX_;
      // Adjust tMaxX to the next X-oriented boundary crossing.
      tMaxX_ += tDeltaX_;
   
  } else {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
  }

  return true;
}


