#include "a_star.h"

Astar::~Astar() {
  for (int i  = 0; i < POOL_SIZE_(0); ++i)
    for (int j = 0; j < POOL_SIZE_(1); ++j)
      for (int k = 0; k < POOL_SIZE_(2); ++k) 
        delete GridNodeMap_[i][j][k];
}

void Astar::initOccMap(env::OccMap::Ptr occ_map, const Eigen::Vector3i pool_size) {
  POOL_SIZE_ = pool_size;
  CENTER_IDX_ = pool_size / 2;
  
  GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
  for (int i = 0; i< POOL_SIZE_(0); ++i) {
    GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
    for (int j = 0; j < POOL_SIZE_(1); ++j) {
      GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
      for (int k = 0; k < POOL_SIZE_(2); ++k) {
        GridNodeMap_[i][j][k] = new GridNode;
      }
    }
  }
  grid_map_ = occ_map;
}

inline double Astar::getHeu(GridNodePtr node1, GridNodePtr node2) {
  return tie_breaker_ * heu->getDiagonalHeu(node1, node2);
} 

std::vector<GridNodePtr> Astar::retrievePath(GridNodePtr currentPtr) {
  std::vector<GridNodePtr> path;
  path.push_back(currentPtr);

  while (currentPtr->cameFrom != NULL) {
      currentPtr = currentPtr->cameFrom;
      path.push_back(currentPtr);
  }

  return path;
}

bool Astar::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    int occ;
    Eigen::Vector3d pos;
    Index2Coord(start_idx, pos);
    if (CheckOccupancy(pos))
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!Coord2Index(start_pt, start_idx))
            {
                return false;
            }
            Index2Coord(start_idx, pos);
            occ = CheckOccupancy(pos);
            if (occ == -1)
            {
                ROS_WARN("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }
    Index2Coord(end_idx, pos);
    if (CheckOccupancy(pos))
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!Coord2Index(end_pt, end_idx))
            {
                return false;
            }
            Index2Coord(start_idx, pos);
            occ = CheckOccupancy(pos);
            if (occ == -1)
            {
                ROS_WARN("[Astar] End point outside the map region.");
                return false;
            }
            Index2Coord(end_idx, pos);
        } while (CheckOccupancy(pos));
    }

    return true;
}

GRAPH_RET Astar::AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt){
  ros::Time t1 = ros::Time::now();
  ++rounds_;
  
  step_size_ = step_size;
  inv_step_size_ = 1.0 / step_size;
  center_ = (start_pt + end_pt) / 2;

  Eigen::Vector3i start_idx, end_idx;
  if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx)) {
    ROS_ERROR("Unable to handle the initial or end point, force return ! ");
    return GRAPH_RET::INIT_ERR;
  }

  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  double tentative_gScore;
  bool flag_explored = false;

  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
  OpenSet_.swap(empty);

  GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
  GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];
  
  endPtr->index = end_idx;

  startPtr->index = start_idx;
  startPtr->gScore = 0;
  startPtr->fScore = getHeu(startPtr, endPtr);
  startPtr->round = rounds_;
  startPtr->state = GridNode::OPENSET;
  startPtr->cameFrom = NULL;

  OpenSet_.push(startPtr);

  int num_iter = 0;

  while(!OpenSet_.empty()) {
    ++num_iter;
    currentPtr = OpenSet_.top();
    OpenSet_.pop();

    if (currentPtr->index(0)==endPtr->index(0) && currentPtr->index(1)==endPtr->index(1) && currentPtr->index(2)==endPtr->index(2)) {
      gridpath_ =  retrievePath(currentPtr);
      ros::Time t2 = ros::Time::now();
      ROS_WARN("[AStar]{sucess} Time in AStar is %f ms, path cost if %f m", (t2 - t1).toSec() * 1000.0, currentPtr->gScore);    
      return GRAPH_RET::SUCCESS;
    } 

    currentPtr->state = GridNode::CLOSEDSET;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {

          if (dx == 0 && dy == 0 && dz == 0) continue;
          
          int nx = (currentPtr->index)(0) + dx;
          int ny = (currentPtr->index)(1) + dy;
          int nz = (currentPtr->index)(2) + dz;
          
          if (nx < 1 || nx >= POOL_SIZE_(0)-1 || ny < 1 || ny >= POOL_SIZE_(1)-1 || nz < 1 || nz >= POOL_SIZE_(2)-1)
            continue;

          neighborPtr = GridNodeMap_[nx][ny][nz];
          neighborPtr->index = Eigen::Vector3i(nx, ny, nz);
          
          // if (CheckOccupancy(Index2Coord(neighborPtr->index))) {
          //   continue;
          // }

          flag_explored = neighborPtr->round == rounds_;

          if (flag_explored && neighborPtr->state==GridNode::CLOSEDSET) {
            continue;
          }

          neighborPtr->round = rounds_;
          
          // optimize: change order between checkoccpancy(consume time) and flag_explored
          Eigen::Vector3d pos;
          Index2Coord(neighborPtr->index, pos);
          if (CheckOccupancy(pos)) {
            continue;
          }

          double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
          tentative_gScore = currentPtr->gScore + static_cost;

          // TODO: update OpenSet_ and neighborPtr
        }
      }
    }
    ros::Time t2 = ros::Time::now();
    if ((t2 - t1).toSec() > 0.2)
    {
        ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
        return GRAPH_RET::SEARCH_ERR;
    }
  }
  ros::Time t2 = ros::Time::now();

  // if ((t2 - t1).toSec() > 0.1)
  ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (t2 - t1).toSec(), num_iter);

  return GRAPH_RET::SEARCH_ERR;
}

std::vector<Eigen::Vector3d> Astar::getPath() {
  std::vector<Eigen::Vector3d> path;
  for (auto node: gridpath_) {
    Eigen::Vector3d pos;
    Index2Coord(node->index, pos);
    path.push_back(pos);
  }
  reverse(path.begin(), path.end());
  return path;
}