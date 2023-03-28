#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

AstarPathFinder::AstarPathFinder() {
    std::string use_tie = use_Tie_breaker? "yes": "no";
    ROS_WARN("[ASTAR NODE] Is Use Tie Break ? %s", use_tie.c_str());
    std::string heu_func;
    switch (select_func)
    {
    case _Euclidean:
        heu_func = "Euclidean";
        break;
    case _Manhattan:
        heu_func = "Manhattan";
        break;
    case _L_infty:
        heu_func = "L_infty";
        break;
    case _Diagonal:
        heu_func = "Diagonal";
        break;
    default:
        heu_func = "NoUseHeu";
        break;
    }
    ROS_WARN("[ASTAR NODE] Which heuristic function is used ? %s", heu_func.c_str());
}

double AstarPathFinder::TieBreaker(GridNodePtr cur_node, GridNodePtr start_node, GridNodePtr end_node) {
    Vector3d cur_coord = cur_node->coord;
    Vector3d start_coord = start_node->coord;
    Vector3d end_coord = end_node->coord;
    Vector3d se = end_coord - start_coord;
    Vector3d ce = end_coord - cur_coord;
    /*
    double dx1 = abs(cur_coord[0] - end_coord[0]);
    double dy1 = abs(cur_coord[1] - end_coord[1]);
    double dz1 = abs(cur_coord[2] - end_coord[2]);

    double dx2 = abs(start_coord[0] - end_coord[0]);
    double dy2 = abs(start_coord[1] - end_coord[1]);
    double dz2 = abs(start_coord[2] - end_coord[2]);
    */
    double cross = 1 - abs(se.dot(ce)/(ce.norm()+0.001)/(se.norm()+0.001));
    return cross;
}

/***************************************************************
  *  @brief     函数作用： 1.设置栅格地图的边界（gl_xl, gl_xu）
  *                      2. GridNodeMap[i][j][k]为栅格地图中每个栅格匹配点云地图的点云坐标
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

/***************************************************************
  *  @brief     函数作用： 重置某个栅格
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

/***************************************************************
  *  @brief     函数作用： 重置栅格地图
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

/***************************************************************
  *  @brief     函数作用： 设置障碍物为1，并使用data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z]标记
  *  @param     参数     
  *  @note      备注      idx为（点云坐标 - 地图下界（负值））/ 分辨率 = 栅格id
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

/***************************************************************
  *  @brief     函数作用： 便利栅格id为-1（闭集合，即被访问过）的点
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

/***************************************************************
  *  @brief     函数作用： 栅格地图坐标转换到点云地图坐标
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

/***************************************************************
  *  @brief     函数作用： 将点云地图坐标转换成栅格地图
  *  @param     参数     
  *  @note      备注      栅格地图的id都是>0的，点云地图的坐标是-x/2到x/2
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

/***************************************************************
  *  @brief     函数作用： 将点云坐标转int（点云坐标转栅格，栅格转点云）
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

/***************************************************************
  *  @brief     函数作用： 访问data，查询是否是障碍物
  *  @param     参数      index：栅格地图的idx索引
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

/***************************************************************
  *  @brief     函数作用： 当前栅格idx是否可以经过
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

/***************************************************************
  *  @brief     函数作用： 1.找到当前栅格所有可以被访问的邻居
  *                      2.获得邻居的
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*finish AstarPathFinder::AstarGetSucc yourself */
    Eigen::Vector3i next_idx;
    Eigen::Vector3i cur_idx = currentPtr->index; 
    GridNodePtr NeighborPtr = NULL;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            for (int k = -1; k <= 1; ++k) {
                if (i==0 && j==0 && k==0)
                    continue;
                next_idx[0] = cur_idx[0] + i;
                next_idx[1] = cur_idx[1] + j;
                next_idx[2] = cur_idx[2] + k;
                if (next_idx[0]<0 || next_idx[0]>GLX_SIZE-1 || next_idx[1]<0 || next_idx[1]>GLY_SIZE || next_idx[2]<0 || next_idx[2]>GLZ_SIZE
                    || isOccupied(next_idx))
                    continue;
                NeighborPtr = GridNodeMap[next_idx[0]][next_idx[1]][next_idx[2]];
                Vector3d next_coord = NeighborPtr->coord;
                Vector3d cur_coord  = currentPtr->coord;
                double dis = std::sqrt(std::pow(next_coord[0]-cur_coord[0], 2) + 
                                       std::pow(next_coord[1]-cur_coord[1], 2) +
                                       std::pow(next_coord[2]-cur_coord[2], 2));
                neighborPtrSets.push_back(NeighborPtr);
                edgeCostSets.push_back(dis);
                
            }
}

/***************************************************************
  *  @brief     函数作用： 1.启发式函数
  *                      2.
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    tie_breaker add it here ?
    finish the AstarPathFinder::getHeu , which is the heuristic function
    *
    */
    double h;
    Eigen::Vector3i start_index = node1->index;
    Eigen::Vector3i end_index   = node2->index;
    switch(select_func) {
        case _Euclidean: {
            // printf("get Euclidean heuristic function...");
            double dx = abs((double)(start_index(0) - end_index(0)));
            double dy = abs((double)(start_index(1) - end_index(1)));
            double dz = abs((double)(start_index(2) - end_index(2)));
            h = std::sqrt(std::pow(dx, 2.0) + std::pow(dy, 2.0) + std::pow(dz, 2.0));
            break;
        }
        case _Manhattan: {
            double dx = abs((double)(start_index(0) - end_index(0)));
            double dy = abs((double)(start_index(1) - end_index(1)));
            double dz = abs((double)(start_index(2) - end_index(2)));
            h = dx + dy + dz;
            break;
        }
        case _L_infty: {
            double dx = abs((double)(start_index(0) - end_index(0)));
            double dy = abs((double)(start_index(1) - end_index(1)));
            double dz = abs((double)(start_index(2) - end_index(2)));
            h = std::max({dx,dy,dz});
            break;
        }
        case _Diagonal: {
            double distance[3];
            distance[0] = abs((double)(start_index(0) - end_index(0)));
            distance[1] = abs((double)(start_index(1) - end_index(1)));
            distance[2] = abs((double)(start_index(2) - end_index(2)));
            std::sort(distance,distance+3);
            h = distance[0] + distance[1] + distance[2] +(std::sqrt(3.0)-3) * distance[0] + (std::sqrt(2.0)-2)*distance[1];
            break;
        }
        default:
            break;
    }
    return h;
}

/***************************************************************
  *  @brief     函数作用： A*算法
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);
    startPtr -> fScore += use_Tie_breaker? 0.001*TieBreaker(startPtr, startPtr, endPtr):0;   
    //finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    some else preparatory works which should be done before while loop
    */
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]]->id = 1;

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    Eigen::Vector3i cur_idx;

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        Remove the node with lowest cost function from open set to closed set
        */
        // remove the node with minimal f value
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());

        cur_idx = currentPtr->index;
        GridNodeMap[cur_idx[0]][cur_idx[1]][cur_idx[2]] -> id = -1;// update the id in grid node map

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }

        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  
        /*For all unexpanded neigbors "m" of node "n", please finish this for loop*/         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            Judge if the neigbors have been expanded
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /* As for a new node, do what you need do ,and then put neighbor in open set and record it */

                // shall update: gScore = inf; fScore = inf; cameFrom = NULL, id, mayby direction
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                neighborPtr->fScore += use_Tie_breaker? 0.001*TieBreaker(neighborPtr, startPtr, endPtr):0; 
                neighborPtr->cameFrom = currentPtr; // todo shallow copy or deep copy
                // push node "m" into OPEN
                openSet.insert(make_pair(neighborPtr -> fScore, neighborPtr));
                neighborPtr -> id = 1;
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it*/
                // shall update: gScore; fScore; cameFrom, mayby direction
                if (neighborPtr -> gScore > (currentPtr -> gScore + edgeCostSets[i])) {
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr,endPtr);
                    neighborPtr -> fScore += use_Tie_breaker? 0.001*TieBreaker(neighborPtr, startPtr, endPtr):0;
                    neighborPtr -> cameFrom = currentPtr;
                }
                continue;
            }
            else{//this node is in closed set
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


/***************************************************************
  *  @brief     函数作用： 得到最优路径经过的点
  *  @param     参数     
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*trace back from the curretnt nodePtr to get all nodes along the path*/
    auto ptr = terminatePtr;
    while(ptr -> cameFrom != NULL){
        gridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}