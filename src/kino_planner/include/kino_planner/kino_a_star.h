#ifndef __KINO_A_STAR__
#define __KINO_A_STAR__
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <occ_grid/occ_map.h>
#include "kino_planner/utils.h"

class KinodynamicAstar {
  private:
    /* -------- main data structure ---------*/
    Eigen::Vector3i POOL_SIZE_, CENTER_IDX_;
    Eigen::Vector3d center_;
    // NodeHashTable expanded_nodes_;  // which sim rounds
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> OpenSet_;
    std::vector<GridNodePtr> gridpath_;
    GridNodePtr ***GridNodeMap_;
    int rounds_{0};
    Heuristics *heu;
    const double tie_breaker_ = 1.0 + 1.0 / 10000;

    /*--------- record data ---------*/
    Eigen::Vector3d start_vel_, end_vel_, start_acc_;
    Eigen::Matrix<double, 6, 6> phi_;
    env::OccMap::Ptr grid_map_;
    Eigen::MatrixXd coef_shot_;
    double t_shot_;
    bool is_shot_succ_ = false;

    /* -------- parameter ----------*/
    /* search */
    double max_tau_, init_max_tau_;
    double max_vel_, max_acc_;
    double w_time_, horizon_, lambda_heu_;
    int allocate_num_, check_num_;
    bool optimistic_;

    /* map */
    double resolution_, inv_resolution_;
    double time_resolution_, inv_time_resolution_;
    double time_origin_;

    /*--------- function ---------*/
    bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);
    
    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &idx) const;
    
    inline bool Coord2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &idx) const;
    
    int Time2Index(double time);
    
    int CheckOccupancy(const Eigen::Vector3d &pos) { return grid_map_->getInflateOccupancy(pos); }
    bool CheckInMap(const Eigen::Vector3i &idx) { return grid_map_->isInMap(idx); }
    bool CheckInMap(const Eigen::Vector3d &pos) { return grid_map_->isInMap(pos); }
    
    std::vector<GridNodePtr> retrievePath(GridNodePtr current);
    
    std::vector<double> cubic(double a, double b, double c, double d);
    
    std::vector<double> quartic(double a, double b, double c, double d, double e);
    
    bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
    
    inline double getHeu(Eigen::VectorXd state1, Eigen::VectorXd state2, double &optimal_time);
    
    void stateTransit(Eigen::Matrix<double, 6, 1> &state0,
                      Eigen::Matrix<double, 6, 1> &state1,
                      Eigen::Vector3d um, double tau);
    

  public:
    typedef shared_ptr<KinodynamicAstar> Ptr;
    KinodynamicAstar(){};
    ~KinodynamicAstar();

    void initParam(ros::NodeHandle &nh);

    void initOccMap(env::OccMap::Ptr occ_map, const Eigen::Vector3i pool_size);
    
    KINO_RET KinoAstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                              Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                              Eigen::Vector3d end_vel, bool init, const double step_size,
                              bool dynamic=false, double start_time=-1.0);
    
    std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
    
    std::vector<Eigen::Vector3d> getPath();

    void getSamples(double &ts, std::vector<Eigen::Vector3d> &point_set,
                    std::vector<Eigen::Vector3d> &start_end_derivatives);

};

inline Eigen::Vector3d KinodynamicAstar::Index2Coord(const Eigen::Vector3i &idx) const{
  return ((idx - CENTER_IDX_).cast<double>() * resolution_) + center_;
}

inline bool KinodynamicAstar::Coord2Index(const Eigen::Vector3d &pts, Eigen::Vector3i &idx) const{
  idx = ((pts - center_) * inv_resolution_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;
  if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2)) {
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}
  return true;
}

#endif