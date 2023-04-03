#include "jps.h"

JPS::JPS () {
    jn3d = new JPS3DNeib();
}

void JPS::initOccMap(env::occ_grid::Ptr occ_map, const Eigen::Vector3i pool_size) {
    POOL_SIZE_ = pool_size;
    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); ++i) {
        GridNodeMap_[i] = new GirdNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); ++j) {
            GridNodeMap_[i][j] = new GridNodePtr [POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); ++k) {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }
    occ_map_ = grid_map;
}

inline void JPS::JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    int num_neib  = jn3d->nsz[norm1][0];
    int num_fneib = jn3d->nsz[norm1][1];
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);

    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
        Eigen::Vector3i neighborIdx;
        Eigen::Vector3i expandDir;

        if( dev < num_neib ) {
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            
            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
                continue;
        }
        else {
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            Eigen::Vector3d pos;
            Eigen::Vector3i idx(nx, ny, nz);
            Index2Coord(idx, pos);
            if(CheckOccupancy(pos)) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
                    continue;
            }
            else
                continue;
        }

        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        nodePtr->dir = expandDir;
        
        neighborPtrSets.push_back(nodePtr);
        edgeCostSets.push_back(
            sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
            );
    }
}

bool JPS::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
{
    neiIdx = curIdx + expDir;

    Eigen::Vector3d neiPos;
    Index2Coord(neiIdx, neiPos);
    if(CheckOccupancy(neiPos))
        return false;

    if( neiIdx == goalIdx )
        return true;

    if( hasForced(neiIdx, expDir) )
        return true;

    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];

    for( int k = 0; k < num_neib - 1; ++k ){
        Eigen::Vector3i newNeiIdx;
        Eigen::Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        if( jump(neiIdx, newDir, newNeiIdx) ) 
            return true;
    }

    return jump(neiIdx, expDir, neiIdx);
}

inline bool JPS::hasForced(const Vector3i & idx, const Vector3i & dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    switch(norm1){
        case 1:
            // 1-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                Eigen::Vector3d pos;
                Eigen::Vector3i idx(nx, ny, nz);
                Index2Coord(idx, pos);
                if(CheckOccupancy(pos))
                    return true;
            }
            return false;

        case 2:
            // 2-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                Eigen::Vector3d pos;
                Eigen::Vector3i idx(nx, ny, nz);
                Index2Coord(idx, pos);
                if(CheckOccupancy(pos))
                    return true;
            }
            return false;

        case 3:
            // 3-d move, check 6 neighbors
            for( int fn = 0; fn < 6; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                Eigen::Vector3d pos;
                Eigen::Vector3i idx(nx, ny, nz);
                Index2Coord(idx, pos);
                if(CheckOccupancy(pos))
                    return true;
            }
            return false;

        default:
            return false;
    }
}

inline double JPS::getHeu(GridNodePtr node1, GridNodePtr node2) {
  return tie_breaker_ * getDiagHeu(node1, node2);
} 

double JPS::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

std::vector<GridNodePtr> JPS::retrievePath(GridNodePtr currentPtr) {
  vector<GridNodePtr> path;
  path.push_back(currentPtr);

  while (currentPtr->cameFrom != NULL) {
      currentPtr = currentPtr->cameFrom;
      path.push_back(currentPtr);
  }

  return path;
}

bool JPS::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
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

void JPS::JpsSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    ++rounds_;
    
    ros::Time t1 = ros::Time::now();    

    //index of start_point and end_point
    Eigen::Vector3i start_idx, end_idx;

    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx)) {
        ROS_ERROR("Unable to handle the initial or end point, force return ! ");
        return JPS_RET::INIT_ERR;
    }

    goalIdx = end_idx; // for jump 'jump' function

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr   = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    //openSet is the open_list implemented through multimap in STL library
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    OpenSet_.swap(empty);
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    endPtr -> index = end_idx;

    //put start node in open set
    startPtr -> index = start_idx;
    startPtr -> state = GridNode::OPENSET;
    startPtr -> round = rounds_;
    startPtr -> cameFrom = NULL;
    //finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr, endPtr);   

    OpenSet_.push_back(startPtr);
    
    /*some else preparatory works which should be done before while loop*/
    double tentative_gScore;
    std::vector<GridNodePtr> neighborPtrSets;
    std::vector<double> edgeCostSets;

    bool flag_explored = false;
    int iter_num = 0;
    // this is the main loop
    while ( !OpenSet_.empty() ){
        ++iter_num;
        /*Remove the node with lowest cost function from open set to closed set*/
        currentPtr = OpenSet_.top();
        OpenSet_.pop();

        // if the current node is the goal 
        if( currentPtr->index == endPtr->index ){
            gridpath_ =  retrievePath(currentPtr);
            ros::Time t2 = ros::Time::now();
            ROS_WARN("[JPS]{sucess} Time in JPS is %f ms, path cost if %f m", (t2 - t1).toSec() * 1000.0, currentPtr->gScore * resolution );    
            return JPS_RET::SUCCESS;
        }
        currentPtr->state = GridNode::CLOSEDSET;

        //get the succetion
        JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets);
        
        /*For all unexpanded neigbors "m" of node "n", please finish this for loop*/         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            Judge if the neigbors have been expanded
            neighborPtrSets[i]->id = -1 : unexpanded
            neighborPtrSets[i]->id = 1 : expanded, equal to this node is in close set
            *        
            */
            neighborPtr = neighborPtrSets[i];

            flag_explored = neighborPtr->round == rounds_;

            if (flag_explored && neighborPtr->state==GridNode::CLOSEDSET)
                continue;
            
            neighborPtr->round = rounds_;

            tentative_gScore = currentPtr -> gScore + edgeCostSets[i];
            if(!flag_explored){ //discover a new node // todo, modified to make it same as A* origianlly (id != 1)
                /* As for a new node, do what you need do ,and then put neighbor in open set and record it*/
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->state = GridNode::OPENSET;

                //update the expanding direction
                for(int i = 0; i < 3; i++){
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if( neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                }
                // push node "m" into OPEN
                OpenSet_.push_bach(neighborPtr);
            }
            else if(neighborPtr-> gScore > tentative_gScore){ //in open set and need update
                /* As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it*/
                neighborPtr -> gScore = tentative_gScore;
                neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr,endPtr);
                neighborPtr -> cameFrom = currentPtr;

                // todo: check the code below
                // if change its parents, update the expanding direction
                //THIS PART IS ABOUT JPS, you can ignore it when you do your Astar work
                for(int i = 0; i < 3; i++){
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if( neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                }
            }      
        }
        ros::Time t2 = ros::Time::now();
        if ((t2 - t1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A JPS path searching !!! 0.2 seconds time limit exceeded.");
            return JPS_RET::SEARCH_ERR;
        }
    }
    //if search fails
    ros::Time t2 = ros::Time::now();
    if((t2 - t1).toSec() > 0.1)
        ROS_WARN("Time consume in JPS path finding is %f", (t2 - t1).toSec() );
}

std::vector<Eigen::Vector3d> JPS::getPath() {
  std::vector<Eigen::Vector3d> path;
  for (auto node: gridpath_) {
    Eigen::Vector3d pos;
    Index2Coord(node->index, pos);
    path.push_back(pos);
  }
  reverse(path.begin(), path.end());
  return path;
}