#include <navigation/global_path_planning.h>

GlobalPathPlanning::GlobalPathPlanning()
{
    // ** Read map graph from file
    Graph_Utils::readGraph(RAS_Names::TOPOLOGICAL_MAP_PATH, this->map_graph_);

    // ** Create DFS planner
    dfs_planner_ = DFS_Planner(this->map_graph_);

    // ** Compute objects graph by BFS on the node
    this->objectGraphFromMap(this->map_graph_, this->objects_graph_);
}

void GlobalPathPlanning::getGlobalPath(std::vector<Node> &path)
{
    // ** Run genetic algorithm to solve TSP for object nodes
    GeneticAlgorithm object_path_planner(this->objects_graph_);
    std::vector<Node> objects_path;
    object_path_planner.computeSolution(objects_path);

    // ** Get sequence of map nodes connecting object nodes
    this->getConnectedObjectsPath(objects_path, path);
}

void GlobalPathPlanning::getConnectedObjectsPath(const std::vector<Node> &objects_path,
                                                       std::vector<Node> &out_path)
{
    // ** Connect each node with the following one using intermediate map (non-object) nodes
    for(std::size_t i = 0; i < objects_path.size() - 1; ++i)
    {
        std::vector<Node> sub_path;
        const Node &start = objects_path[i];
        const Node &end   = objects_path[i+1];

        dfs_planner_.computePath(start, end, sub_path);

        bool is_last_segment = (i == (objects_path.size()-1));
        this->addSubpath(sub_path, is_last_segment, out_path);
    }
}

void GlobalPathPlanning::addSubpath(const std::vector<Node> &subpath, bool last_segment,
                                          std::vector<Node> &out_path)
{
    // Don't add the last element since it will be added in the next segment (unless
    // it's the last one)
    std::size_t n_elems = subpath.size();
    std::size_t end = last_segment ? n_elems : n_elems -1;

    for(std::size_t i = 0; i < end; ++i)
    {
        out_path.push_back(subpath[i]);
    }
}

void GlobalPathPlanning::objectGraphFromMap(const Graph &map_graph, Graph &objects_graph)
{
    const std::vector<Node> &map_nodes = map_graph.getNodes();
    std::vector<Node> object_nodes;
    std::vector<Edge> object_edges;

    for(std::size_t i = 0; i < map_nodes.size() - 1; ++i)
    {
        for(std::size_t j = i; j < map_nodes.size(); ++j)
        {
            const Node &n1 = map_nodes[i];
            const Node &n2 = map_nodes[j];

            if(n1.isObject() < 0 && n2.isObject() < 0) // Both are objects
            {
                // ** Push objects into the vector
                if(std::find(object_nodes.begin(), object_nodes.end(), n1) != object_nodes.end())
                    object_nodes.push_back(n1);

                if(std::find(object_nodes.begin(), object_nodes.end(), n2) != object_nodes.end())
                    object_nodes.push_back(n2);

                std::vector<Node> sub_path;
                double cost = dfs_planner_.computePath(n1, n2, sub_path);

                // ** Create edge and push into the vector
                object_edges.push_back(Edge (n1, n2, cost));
            }
        }
    }
    // ** Update graph
    objects_graph = Graph(object_nodes, object_edges);
}
