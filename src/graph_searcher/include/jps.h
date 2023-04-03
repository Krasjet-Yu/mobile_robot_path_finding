#ifndef _JPS_SEARCHER_H_
#define _JPS_SEARCHER_H_

#include <iostream>
#include <queue>
#include <Eigen/Eigen>
#include "ros/ros.h"
#include "utils.h"
#include "occ_grid/occ_map.h"

enum JPS_RET {
  SUCCESS,
  INIT_ERR,
  SEARCH_ERR
};

class JPS {	
	private:
		env::OccMap::Ptr grid_map_;

		Eigen::Vector3d center_;
		Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
		
		int rounds_{0};
		double step_size_, inv_step_size_;
		Eigen::Vector3i goalIdx;
		const double tie_breaker_ = 1.0 + 1.0 / 10000;

		std::vector<GridNodePtr> gridpath_;
		GridNodePtr ***GridNodeMap_;
		std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> OpenSet_;

		inline double getHeu(GridNodePtr node1, GridNodePtr node2);

  	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);

  	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

		inline bool Coord2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &idx) const;

		inline bool Index2Coord(const Eigen::Vector3i &idx, Eigen::Vector3d &pos) const;

		inline int CheckOccupancy(const Eigen::Vector3d &pos) {return grid_map_->getInflateOccupancy(pos);}

		std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	public:
		JPS3DNeib * jn3d;

    JPS();
    	
    ~JPS() { delete jn3d; };

		void initOccMap(env::OccMap::Ptr occ_map, const Eigen::Vector3i pool_size);

		void JPSGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
    
		bool hasForced(const Eigen::Vector3i & idx, const Eigen::Vector3i & dir);
    
		bool jump(const Eigen::Vector3i & curIdx, const Eigen::Vector3i & expDir, Eigen::Vector3i & neiIdx);
		
    JPS_RET JpsSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

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