#ifndef GLOBAL_PATH_PLANNING_H
#define GLOBAL_PATH_PLANNING_H

#include <ras_utils/graph/graph.h>
#include <ras_utils/genetic/genetic_algorithm.h>
#include <ras_utils/graph/dfs_planner.h>
#include <algorithm>
#include <ras_utils/ras_names.h>

#include <Eigen/Core>

#include <queue>
class GlobalPathPlanning
{
public:
    /**
     * @brief Default constructor. Reads the topological map from file
     */
    GlobalPathPlanning();

    /**
     * @brief Computes the sequence of topological nodes to be visited in order to
     *        go from start (0,0), fetch all the objects, and go back as fast as possible
     * @return
     */
    void getGlobalPath(std::queue<Node> &path);

private:
    Graph map_graph_;         // Topological map of the whole maze, including objects
    Graph objects_graph_;     // Topological map of only objects
    DFS_Planner dfs_planner_; // Path planning in the graph

    void objectGraphFromMap(const Graph &map_graph, Graph &objects_graph);

    void getConnectedObjectsPath(const std::vector<Node> &objects_path,
                                       std::queue<Node> &out_path);
    void addSubpath(const std::vector<Node> &subpath, bool last_segment, std::queue<Node> &out_path);
};

#endif // GLOBAL_PATH_PLANNING_H
