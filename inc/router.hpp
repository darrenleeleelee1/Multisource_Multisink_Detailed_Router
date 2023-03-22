#pragma once
#include <memory>
#include "layout.hpp"
#include "grid.hpp"
class Router
{
public:
    Layout *layout;
    Grid *grid;
    Grid *pin_and_obstacle_grid;
    static const std::vector<std::vector<Coordinate3D>> move_orientation;
    double history_cost = 0.1;
    unsigned num_of_reroute;
    Router() {}
    Router(Layout *l) {
        this->layout = l;
        this->grid = new Grid(l->width, l->height);
        this->pin_and_obstacle_grid = new Grid(this->layout);
        this->num_of_reroute = 0;
        // number of net + 1 means obstacle
        for(auto o : l->obstacles) this->grid->setObstacles(l->netlist.size(), o.start_point, o.end_point);
        for(auto n : l->netlist){
            for(auto p : n.pins) this->grid->setObstacles(n.id, p, p);
        }
    }
    ~Router() {
        delete this->grid;
        delete this->pin_and_obstacle_grid;
    }

    void main();
    void twoPinNetDecomposition();
    bool outOfBound(Coordinate3D p);
    bool tree2treeMazeRouting(Net *net, Subtree *source, Subtree *sink);
    Path tree2treeMazeRouting(Grid *tmp_grid, Net *net, Subtree *source, Subtree *sink);
    void addHistoryCost(Path *p);
    void routing(Net &n, int source_index, int sink_index);
};

std::pair<int, int> ripUpPaths(Grid *grid, Path *rip_up_candidate, Tree *updated_tree);
void mergePaths();

