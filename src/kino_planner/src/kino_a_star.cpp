#include "kino_planner/kino_a_star.h"

KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < POOL_SIZE_(0); ++i)
    for (int j = 0; j < POOL_SIZE_(1); ++j)
      for (int k = 0; k < POOL_SIZE_(2); ++k)
        delete GridNodeMap_[i][j][k];
}

void KinodynamicAstar::initParam(ros::NodeHandle &nh) {
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/optimistic", optimistic_, true);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  inv_time_resolution_ = 1 / time_resolution_;
  nh.param("search/check_num", check_num_, -1);
  double vel_margin;
  nh.param("search//vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;
  /*
  1 & 0 & 0 & 0 & 0 & 0 \\
  0 & 1 & 0 & 0 & 0 & 0 \\
  0 & 0 & 1 & 0 & 0 & 0 \\
  0 & 0 & 0 & 1 & 0 & 0 \\
  0 & 0 & 0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 0 & 0 & 1 \\
  */
  phi_ = Eigen::MatrixXd::Identity(6, 6);
  ROS_INFO("[INIT] Init Parameter finish ! ");
}

void KinodynamicAstar::initOccMap(env::OccMap::Ptr occ_map, const Eigen::Vector3i pool_size) {
  POOL_SIZE_ = pool_size;
  CENTER_IDX_ = pool_size / 2;
  GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
  for (int i = 0; i < POOL_SIZE_(0); ++i) {
    GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
    for (int j = 0; j < POOL_SIZE_(1); ++j) {
      GridNodeMap_[i][j] = new GridNodePtr [POOL_SIZE_(2)];
      for (int k = 0; k < POOL_SIZE_(2); ++k) {
        GridNodeMap_[i][j][k] = new GridNode;
      }
    }
  }
  grid_map_ = occ_map;
  ROS_INFO("[INIT] Init Occ Map finish ! ");
}

bool KinodynamicAstar::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    int occ;
    Eigen::Vector3d pos = Index2Coord(start_idx);
    if (CheckOccupancy(pos))
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * resolution_ + start_pt;
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!Coord2Index(start_pt, start_idx))
            {
                return false;
            }
            pos = Index2Coord(start_idx);
            occ = CheckOccupancy(pos);
            if (occ == -1)
            {
                ROS_WARN("[Kinodynamic Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }
    pos = Index2Coord(end_idx);
    if (CheckOccupancy(pos))
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * resolution_ + end_pt;
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!Coord2Index(end_pt, end_idx))
            {
                return false;
            }
            pos = Index2Coord(start_idx);
            occ = CheckOccupancy(pos);
            if (occ == -1)
            {
                ROS_WARN("[Kinodynamic Astar] End point outside the map region.");
                return false;
            }
            pos = Index2Coord(end_idx);
        } while (CheckOccupancy(pos));
    }

    return true;
}

inline double KinodynamicAstar::getHeu(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                       double& optimal_time) {
  const Eigen::Vector3d dp = state2.head(3) - state1.head(3);
  const Eigen::Vector3d v0 = state1.segment(3, 3);
  const Eigen::Vector3d v1 = state2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (state1.head(3) - state2.head(3)).lpNorm<Eigen::Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
} 

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                       double time_to_goal) {
  /* ---------- get coefficient ---------- */
  const Eigen::Vector3d p0 = state1.head(3);
  const Eigen::Vector3d dp = state2.head(3) - p0;
  const Eigen::Vector3d v0 = state1.segment(3, 3);
  const Eigen::Vector3d v1 = state2.segment(3, 3);
  const Eigen::Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  Eigen::MatrixXd coef(3, 4);
  end_vel_ = v1;

  Eigen::Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Eigen::Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Eigen::Vector3d c = v0;
  Eigen::Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Eigen::Vector3d coord, vel, acc;
  Eigen::VectorXd poly1d, t, polyv, polya;
  Eigen::Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = Eigen::VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++) {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (std::fabs(vel(dim)) > max_vel_ || std::fabs(acc(dim)) > max_acc_) {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (!CheckInMap(coord)) {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    if (CheckOccupancy(coord)) {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

std::vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d) {
  std::vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

std::vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e) {
  
  // ref: https://zhuanlan.zhihu.com/p/491057733#ref_2

  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                                    Eigen::Matrix<double, 6, 1>& state1, 
                                    Eigen::Vector3d um, double tau) {
  /*
  1 & 0 & 0 & t & 0 & 0 \\
  0 & 1 & 0 & 0 & t & 0 \\
  0 & 0 & 1 & 0 & 0 & t \\
  0 & 0 & 0 & 1 & 0 & 0 \\
  0 & 0 & 0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 0 & 0 & 1 \\
  */
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t) {
  std::vector<Eigen::Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  GridNodePtr node = gridpath_.back();
  Eigen::Matrix<double, 6, 1> x0, xt;

  while (node->cameFrom != NULL) {
    Eigen::Vector3d ut = node->uav_input;
    double duration = node->duration;
    x0 = node->cameFrom->uav_state;

    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->cameFrom;
  }
  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_) {
    Eigen::Vector3d coord;
    Eigen::VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives) {
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_) T_sum += t_shot_;
  GridNodePtr node = gridpath_.back();
  while (node->cameFrom != NULL) {
    T_sum += node->duration;
    node = node->cameFrom;
  }
  // cout << "duration:" << T_sum << endl;

  // Calculate boundary vel and acc
  Eigen::Vector3d end_vel, end_acc;
  double t;
  if (is_shot_succ_) {
    t = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim) {
      Eigen::Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  } else {
    t = gridpath_.back()->duration;
    end_vel = node->uav_state.tail(3);
    end_acc = gridpath_.back()->uav_input;
  }

  // Get point samples
  int seg_num = floor(T_sum / ts);
  seg_num = max(8, seg_num);
  ts = T_sum / double(seg_num);
  bool sample_shot_traj = is_shot_succ_;
  node = gridpath_.back();

  for (double ti = T_sum; ti > -1e-5; ti -= ts) {
    if (sample_shot_traj) {
      // samples on shot traj
      Eigen::Vector3d coord;
      Eigen::Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      if (t < -1e-5) {
        sample_shot_traj = false;
        if (node->cameFrom != NULL) t += node->duration;
      }
    } else {
      // samples on searched traj
      Eigen::Matrix<double, 6, 1> x0 = node->cameFrom->uav_state;
      Eigen::Matrix<double, 6, 1> xt;
      Eigen::Vector3d ut = node->uav_input;

      stateTransit(x0, xt, ut, t);

      point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->cameFrom->cameFrom != NULL) {
        node = node->cameFrom;
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector3d start_acc;
  if (gridpath_.back()->cameFrom == NULL) {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  } else {
    // input of searched traj
    start_acc = node->uav_input;
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

int KinodynamicAstar::Time2Index(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

KINO_RET KinodynamicAstar::KinoAstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                                            Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, const double step_size,
                                            bool dynamic, double start_time) {
  ros::Time t1 = ros::Time::now();
  ++rounds_;
  
  resolution_ = step_size;
  inv_resolution_ = 1 / step_size;
  center_ = (start_pt + end_pt) / 2;
  start_vel_ = start_v;
  start_acc_ = start_a;

  Eigen::Vector3i start_idx, end_idx;
  if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx)) {
    ROS_ERROR("Unable to handle the initial or end point, force return ! ");
    return KINO_RET::INIT_ERR;
  }

  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  double tentative_gScore, tentative_fScore, time_to_goal;
  bool flag_explored = false;

  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
  OpenSet_.swap(empty);

  GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
  GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];
  
  startPtr->index = start_idx;
  startPtr->uav_state.head(3) = start_pt;
  startPtr->uav_state.tail(3) = start_v;
  startPtr->state = GridNode::OPENSET;
  startPtr->cameFrom = NULL;
  startPtr->gScore = 0.0;
  
  endPtr->index = end_idx;
  endPtr->uav_state.head(3) = end_pt;
  endPtr->uav_state.tail(3) = end_v;
  startPtr->fScore = tie_breaker_ * getHeu(startPtr->uav_state, endPtr->uav_state, time_to_goal);
  OpenSet_.push(startPtr);

  // update round(means visited or expanded) and time_idx
  if (dynamic) {
    time_origin_ = start_time;
    startPtr->round = rounds_;
    startPtr->time = start_time;
    startPtr->time_idx = Time2Index(start_time);
    // expanded_nodes_.insert(startPtr->index, startPtr->time_idx, startPtr);
  }
  else {
    startPtr->round = rounds_;
    // expanded_nodes_.insert(startPtr->index, startPtr);
  }

  bool init_search = init;
  const int tolerance = std::ceil(inv_resolution_);

  int iter_num_ = 0;

  while (!OpenSet_.empty()) {
    ++iter_num_;
    currentPtr = OpenSet_.top();
    OpenSet_.pop();
    currentPtr->state = GridNode::CLOSEDSET;

    bool reach_horizon = (currentPtr->uav_state.head(3) - start_pt).norm() >= horizon_;
    bool near_end = std::abs(currentPtr->index(0) - end_idx(0)) <= tolerance &&
                    std::abs(currentPtr->index(1) - end_idx(1)) <= tolerance &&
                    std::abs(currentPtr->index(2) - end_idx(2)) <= tolerance;
    if (reach_horizon || near_end) {
      gridpath_ = retrievePath(currentPtr);
      if (near_end) {
        // Check whether shot traj exist
        getHeu(currentPtr->uav_state, endPtr->uav_state, time_to_goal);
        computeShotTraj(currentPtr->uav_state, endPtr->uav_state, time_to_goal);
        if (init_search)
          ROS_ERROR("Shot in first search loop!");
      }
    }
    if (reach_horizon) {
      if (is_shot_succ_) {
        std::cout << "reach end" << std::endl;
        return KINO_RET::REACH_END; 
      }
      else {
        std::cout << "reach horizon" << std::endl;
        return KINO_RET::REACH_HORIZON;
      }
    }

    if (near_end) {
      if (is_shot_succ_) {
        std::cout << "reach end" << std::endl;
        return KINO_RET::REACH_END; 
      } 
      else if (currentPtr->cameFrom != NULL) {
        std::cout << "near end" << std::endl;
        return KINO_RET::NEAR_END;
      }
      else {
        std::cout << "no path" << std::endl;
        return KINO_RET::NO_PATH;
      }
    }
    // input resolution
    double res = 1 / 2.0;
    // time resolution
    double time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = currentPtr->uav_state;
    Eigen::Matrix<double, 6, 1> pro_state;
    std::vector<GridNodePtr>  prune_nodes;
    Eigen::Vector3d um; // control input
    double pro_t;
    std::vector<Eigen::Vector3d> inputs;   // input
    std::vector<double> durations;         // time duration
    if (init_search)  {
      // In the initial stage, run at a constant speed
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
          tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      // init_search = false;
    }
    else {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res) {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_) 
        durations.push_back(tau);
    }

    // find and update neighbor
    for (int i = 0; i < inputs.size(); ++i) {
      for (int j = 0; j < durations.size(); ++j) {
        um = inputs[i];
        double tau = durations[j];
        // input um & tau, transit state from cur state to pro state
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = currentPtr->time + tau;
        
        // Check inside map range
        Eigen::Vector3d pro_pos = pro_state.head(3);
        if (!CheckInMap(pro_pos)) {
          if (init_search) std::cout << "outside map range" << std::endl;
          init_search = false;
          continue;
        }

        // Check if in close set
        Eigen::Vector3i pro_idx;
        Coord2Index(pro_pos, pro_idx);
        int pro_t_idx = Time2Index(pro_t);
        neighborPtr = GridNodeMap_[pro_idx(0)][pro_idx(1)][pro_idx(2)];
        neighborPtr->index = pro_idx;
        flag_explored = neighborPtr->round == rounds_;
        if (flag_explored && neighborPtr->time_idx == pro_t_idx && neighborPtr->state == GridNode::CLOSEDSET) {
          if (init_search) std::cout << "close" << std::endl;
          init_search = false;
          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (std::fabs(pro_v(0)) > max_vel_ || std::fabs(pro_v(1)) > max_vel_ || std::fabs(pro_v(2)) > max_vel_) {
          if (init_search) std::cout << "maximum speed exceeded" << std::endl;
          init_search = false;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_idx - currentPtr->index;
        int diff_time = pro_t_idx - currentPtr->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0)) {
          if (init_search) std::cout << "in the same voxel with current node" << std::endl;
          init_search = false;
          continue;
        }

        // Check safety in path
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k) {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          if (CheckOccupancy(pos)) {
            is_occ = true;
            break;
          }
        }
        if (is_occ) {
          if (init_search) std::cout << "occur collision" << std::endl;
          init_search = false;
          continue;
        }

        tentative_gScore = (um.squaredNorm() + w_time_) * tau + currentPtr->gScore;
        tentative_fScore = tentative_gScore + tie_breaker_ * getHeu(pro_state, endPtr->uav_state, time_to_goal);
        
        neighborPtr->round = rounds_;

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (int j = 0; j < prune_nodes.size(); ++j) {
          GridNodePtr prune_node = prune_nodes[j];
          if ((pro_idx - prune_node->index).norm() == 0 && 
              ((!dynamic) || pro_t_idx == prune_node->time_idx)) {
            prune = true;
            if (tentative_fScore < prune_node->fScore) {
              prune_node->fScore = tentative_fScore;
              prune_node->gScore = tentative_gScore;
              prune_node->uav_state = pro_state;
              prune_node->uav_input = um;
              prune_node->duration = tau;
              if (dynamic) prune_node->time = currentPtr->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune) {
          if (!flag_explored) {
            neighborPtr->index = pro_idx;
            neighborPtr->uav_state = pro_state;
            neighborPtr->fScore = tentative_fScore;
            neighborPtr->gScore = tentative_gScore;
            neighborPtr->uav_input = um;
            neighborPtr->duration = tau;
            neighborPtr->cameFrom = currentPtr;
            neighborPtr->state = GridNode::OPENSET;
            if (dynamic) {
              neighborPtr->time = currentPtr->time + tau;
              neighborPtr->time_idx = Time2Index(neighborPtr->time);
            }
            OpenSet_.push(neighborPtr);

            // if (dynamic)
            //   expanded_nodes_.insert(pro_idx, neighborPtr->time, neighborPtr);
            // else
            //   expanded_nodes_.insert(pro_idx, neighborPtr);

            prune_nodes.push_back(neighborPtr);

            // use_node_num_ += 1;
            // if (use_node_num_ == allocate_num_) {
            //   cout << "run out of memory." << endl;
            //   return KINO_RET::NO_PATH;
            // }
          } 
          else if (neighborPtr->gScore > tentative_gScore) {
              // neighborPtr->index = pro_id;
              neighborPtr->uav_state = pro_state;
              neighborPtr->fScore = tentative_fScore;
              neighborPtr->gScore = tentative_gScore;
              neighborPtr->uav_input = um;
              neighborPtr->duration = tau;
              neighborPtr->cameFrom = currentPtr;
              if (dynamic) neighborPtr->time = currentPtr->time + tau;
          } else {
            // std::cout << "error type in searching: " << neighborPtr->state << std::endl;
          }
        }
      }
    }
    ros::Time t2 = ros::Time::now();
    if ((t2 - t1).toSec() > 0.2)
    {
        ROS_WARN("Failed in Kinodynamic A star path searching !!! 0.2 seconds time limit exceeded.");
        return KINO_RET::NO_PATH;
    }
  }
  std::cout << "Open Set Empty, No Path!" << std::endl;
  // std::cout << "use node num: " << use_node_num_ << std::endl;
  // std::cout << "iter num: " << iter_num_ << std::endl;
  ros::Time t2 = ros::Time::now();
  // if ((t2 - t1).toSec() > 0.1)
  ROS_WARN("Time consume in Kinodynamic A star path finding is %.3fs, iter=%d", (t2 - t1).toSec(), iter_num_);

  return KINO_RET::NO_PATH;
}

std::vector<GridNodePtr> KinodynamicAstar::retrievePath(GridNodePtr currentPtr) {
  std::vector<GridNodePtr> path;
  path.push_back(currentPtr);

  while (currentPtr->cameFrom != NULL) {
      currentPtr = currentPtr->cameFrom;
      path.push_back(currentPtr);
  }

  return path;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getPath() {
  std::vector<Eigen::Vector3d> path;
  for (auto node: gridpath_) {
    Eigen::Vector3d pos;
    pos = Index2Coord(node->index);
    path.push_back(pos);
  }
  reverse(path.begin(), path.end());
  return path;
}