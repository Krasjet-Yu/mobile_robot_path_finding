# mobile_robot_path_finding

```python-repl
git clone https://github.com/Krasjet-Yu/mobile_robot_path_finding.git
cd mobile_robot_path_finding
catkin_make
```

## Graph Based Path Finding

```shell
source devel/setup.bash
roslaunch graph_searcher rviz.launch
```

```shell
source devel/setup.bash
roslaunch graph_searcher graph_search.launch
```

## Sample Guid Path Finding
which refers to: https://github.com/ZJU-FAST-Lab/sampling-based-path-finding.git
```shell
source devel/setup.bash
roslaunch sample_guider rviz.launch
```

```shell
source devel/setup.bash
roslaunch sample_guider sample_search.launch
```

## 代码提交须知
1. 开发前，更新Features TODO(记录Feature内容，开发周期)
2. 开发前，从最新的master切出开发分支，分支的格式参照 feature-{date}-{username}-{feature_name}
3. master 分支必须提MR合入，Code Review -> 
4. 提MR前，需保证代码风格统一

## Feature TODO
- Graph Search
  - A*
  - JPS
- Sample Search
  - RRT
  - RRT*
  - RRT#
  - GUILD
- Multi Agent Search
  - CBS
  - ECBS
- Visualization
- Interaction