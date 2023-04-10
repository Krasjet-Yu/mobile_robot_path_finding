#ifndef _UTILS_H_
#define _UTILS_H_

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <utility>
#include <ros/ros.h>
#include <ros/console.h>

constexpr double inf = 1 >> 20;

struct GridNode;
typedef GridNode *GridNodePtr;

enum KINO_RET {
  INIT_ERR=0,
	REACH_HORIZON=1,
  REACH_END=2,
  NO_PATH=3,
  NEAR_END=4
};

enum UGV_INPUT {
  LEFT_FORWORD,
  STRAIGHT_FORWORD,
  RIGHT_FORWORD,
  LEFT_BACKWORD,
  STARIGHT_BACKWORD,
  RIGHT_BACKWORD
};

struct GridNode
{
  enum enum_state {
    OPENSET = 1,
    CLOSEDSET = 2,
    UNDEFIND = 3
  };

  /* init variable */

  enum enum_state state{UNDEFIND};
  Eigen::Vector3i index;
	Eigen::Matrix<double, 6, 1> uav_state;  // px, py, pz, vx, vy, vz
  Eigen::Vector3d ugv_state; // px, py, yaw
  Eigen::Vector3d uav_input; // ax, ay, az
  UGV_INPUT ugv_input;
  
  double duration;
  double time;     //dyn
  int time_idx;
  int round;
  double gScore{inf}, fScore{inf};

  GridNodePtr cameFrom{NULL};
};

struct Heuristics {

	double getDiagonalHeu(GridNodePtr node1, GridNodePtr node2);
	double getEculideanHeu(GridNodePtr node1, GridNodePtr node2);
	double getLinftyHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhattanHeu(GridNodePtr node1, GridNodePtr node2);

};


class NodeComparator {
public:
  bool operator()(GridNodePtr node1, GridNodePtr node2) {
    return node1->fScore > node2->fScore;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
  private:
  /* data */
    std::unordered_map<Eigen::Vector3i, GridNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;  // px, py, pz
    std::unordered_map<Eigen::Vector4i, GridNodePtr, matrix_hash<Eigen::Vector4i>> data_4d_;  // px, py, pz, t
  
  public:
    NodeHashTable() {}
    ~NodeHashTable() {}
    void insert(Eigen::Vector3i idx, GridNodePtr node) {
      data_3d_.insert(std::make_pair(idx, node));
    }
    void insert(Eigen::Vector3i idx, int time_idx, GridNodePtr node) {
      data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
    }
    GridNodePtr find(Eigen::Vector3i idx) {
      auto iter = data_3d_.find(idx);
      return iter == data_3d_.end()? NULL : iter->second;
    }
    GridNodePtr find(Eigen::Vector3i idx, int time_idx) {
      auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
      return iter == data_4d_.end()? NULL : iter->second;
    }
    void clear() {
      data_3d_.clear();
      data_4d_.clear();
    }
};

#endif