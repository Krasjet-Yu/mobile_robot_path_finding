#ifndef _A_STAR_H_
#define _A_STAR_H_
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "occ_grid/occ_map.h"
#include "utils.h"
#include <queue>

enum ASTAR_RET {
  SUCCESS,
  INIT_ERR,
  SEARCH_ERR
};

class Astar{
private:
  env::OccMap::Ptr grid_map_;

  inline double getHeu(GridNodePtr node1, GridNodePtr node2);

  double getDiagHeu(GridNodePtr node1, GridNodePtr node2);

  bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

  inline bool Coord2Index(const Eigen::Vector3d &pts, Eigen::Vector3i &idx) const;

  inline bool Index2Coord(const Eigen::Vector3i &idx, Eigen::Vector3d &pts) const;

  inline int CheckOccupancy(const Eigen::Vector3d &pos) {return grid_map_->getInflateOccupancy(pos);}

  std::vector<GridNodePtr> retrievePath(GridNodePtr current);

  int rounds_{0};
  double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

  std::vector<GridNodePtr> gridpath_;

  GridNodePtr ***GridNodeMap_;

  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> OpenSet_;

public:
  typedef std::shared_ptr<Astar> Ptr;

  Astar(){};
  
  ~Astar();

  void initOccMap(env::OccMap::Ptr occ_map, const Eigen::Vector3i pool_size);

  ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

  std::vector<Eigen::Vector3d> getPath();
};

inline bool Astar::Index2Coord(const Eigen::Vector3i &idx, Eigen::Vector3d &pts) const{
  pts = ((idx - CENTER_IDX_).cast<double>() * step_size_) + center_;
  if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2)) {
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}
  return true;
}

inline bool Astar::Coord2Index(const Eigen::Vector3d &pts, Eigen::Vector3i &idx) const{
  idx = ((pts - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;
  if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2)) {
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}
  return true;
}

#endif