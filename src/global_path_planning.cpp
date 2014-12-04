#include <navigation/global_path_planning.h>

GlobalPathPlanning::GlobalPathPlanning()
{
    // ** Read graph from file
    Graph_Utils::readGraph(RAS_Names::OBJECT_GRAPH_PATH, objects_graph_);
}

void GlobalPathPlanning::getGlobalPath(std::vector<Node> &path)
{
    // ** Run genetic algorithm to solve TSP for object nodes
    GeneticAlgorithm object_path_planner(this->objects_graph_);
    object_path_planner.computeSolution(path);
}
