#include "arena_path_search/astar.h"



Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

void Astar::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

void Astar::setParam(ros::NodeHandle& private_nh) {
  private_nh.param("astar/resolution_astar", resolution_, 0.05);
  private_nh.param("astar/lambda_heu", lambda_heu_, 0.0001);
  private_nh.param("astar/allocate_num", allocate_num_, 100000);
  tie_breaker_ = 1.0 + 1.0 / 10000;
  std::cout << "lambda_heu_:" << lambda_heu_ << std::endl;
}

void Astar::setEnvironment(const GridMap::Ptr& env) {
  this->grid_map_ = env;
}

void Astar::init(){
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  grid_map_->getRegion(origin_, map_size_2d_);

  std::cout << "origin_: " << origin_.transpose() << std::endl;
  std::cout << "map size: " << map_size_2d_.transpose() << std::endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

int Astar::search(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt) {
  /* ---------- initialize ---------- */
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::Vector2i end_index;

  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;

  open_set_.push(cur_node);
  use_node_num_ += 1;

  expanded_nodes_.insert(cur_node->index, cur_node); // insert to hashtable expanded_nodes_
  
  //NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();  

    /* ---------- determine termination ---------- */

    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                        abs(cur_node->index(1) - end_index(1)) <= 1 ;

    if (reach_end) {
      // std::cout << "[Astar]:---------------------- " << use_node_num_ << std::endl;
      // std::cout << "use node num: " << use_node_num_ << std::endl;
      // std::cout << "iter num: " << iter_num_ << std::endl;
      terminate_node = cur_node; 
      retrievePath(terminate_node);
      has_path_ = true;

      return REACH_END;
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */

    Eigen::Vector2d cur_pos = cur_node->position;
    Eigen::Vector2d pro_pos;

    Eigen::Vector2d d_pos;

    /* ---------- expansion loop ---------- */
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_){
          // delta distance
          d_pos << dx, dy;

          if (d_pos.norm() < 1e-3) continue;

          // neight pos
          pro_pos = cur_pos + d_pos;

          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_2d_(0) || 
                pro_pos(1) <= origin_(1) ||pro_pos(1) >= map_size_2d_(1)) {
            std::cout << "outside map" << std::endl;
            continue;
          }

          /* not in close set */
          Eigen::Vector2i pro_id = posToIndex(pro_pos);
          NodePtr pro_node=NULL;

          pro_node =expanded_nodes_.find(pro_id);
          
          
          // if pro_node is expended and is in closeset
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
            //std::cout << "in closeset" << std::endl;
            continue;
          }

          // check if pro_ps is in collision
          int occu = grid_map_->getFusedInflateOccupancy(pro_pos);
          if (occu==1){
            //std::cout << "in collision" << std::endl;
            continue;
          }
          
          // double dist = edt_environment_->evaluateCoarseEDT(pro_pos);
          // if (dist <= margin_) {
          //   std::cout << "in collision" << std::endl;
          //   continue;
          // }

          /* ---------- compute cost ---------- */
          double tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
          tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt); //getEuclHeu

          if (pro_node == NULL) { // pro_node has not been expanded, was not in openset
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            
            open_set_.push(pro_node);

            expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              std::cout << "run out of memory." << std::endl;
              return NO_PATH;
            }
          } else if (pro_node->node_state == IN_OPEN_SET) { // already expended, in openset, just update parent, g & f
            if (tmp_g_score < pro_node->g_score) {
              // pro_node->index = pro_id;
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
            }
          } else {
            std::cout << "error type in searching: " << pro_node->node_state << std::endl;
          }

          /* ----------  ---------- */
        }
  }

  /* ---------- open set empty, no path ---------- */
  std::cout << "[Astar replan]open set empty, no path!" << std::endl;
  std::cout << "[Astar replan]use node num: " << use_node_num_ << std::endl;
  std::cout << "[Astar replan]iter num: " << iter_num_ << std::endl;
  return NO_PATH;
}

void Astar::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double Astar::getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  
  double h;
  int diag = std::min(dx, dy);
  dx -= diag;
  dy -= diag;

  h = 1.0 * sqrt(2.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);

  return tie_breaker_ * h;
}

double Astar::getManhHeu(Eigen::Vector2d x1, Eigen::Vector2d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  

  return tie_breaker_ * (dx + dy);
}

double Astar::getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

Eigen::Vector2i Astar::posToIndex(Eigen::Vector2d pt) {
  Eigen::Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

std::vector<Eigen::Vector2d> Astar::getPath() {
  std::vector<Eigen::Vector2d> path;
  for (unsigned int i = 0; i < path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->position);
  }
  return path;
}

std::vector<NodePtr> Astar::getVisitedNodes() {
  std::vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector2d goalCallback(const geometry_msgs::PoseStampedPtr& msg){
  Eigen::Vector2d end_pt;
  if(msg->pose.position.z !=0) return end_pt;
  
  end_pt(0)=msg->pose.position.x;
  end_pt(1)=msg->pose.position.y;
  return end_pt;
}



