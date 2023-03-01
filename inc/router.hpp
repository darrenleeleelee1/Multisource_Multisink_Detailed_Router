#pragma once
#include <memory>
#include "layout.hpp"
#include "grid.hpp"
class Router
{
public:
    Layout *layout;
    Grid *grid;
    static const std::vector<std::vector<Coordinate3D>> move_orientation;
    Router() {}
    Router(Layout *l) {
        this->layout = l;
        this->grid = new Grid(l->width, l->height);
        // number of net + 1 means obstacle
        for(auto o : l->obstacles) this->grid->setObstacles(l->netlist.size(), o.start_point, o.end_point);
        for(auto n : l->netlist){
            for(auto p : n.pins) this->grid->setObstacles(n.id, p, p);
        }
    }
    ~Router() {
        delete grid;
    }

    void main();
    void twoPinNetDecomposition();
    bool outOfBound(Coordinate3D p);
    bool tree2tree_maze_routing(Net *net, Subtree *source, Subtree *sink);
    Edge tree2tree_maze_routing(Grid *tmp_grid, Net *net, Subtree *source, Subtree *sink);
};

std::pair<int, int> ripUpEdges(Grid *grid, Edge *rip_up_candidate, Tree *updated_tree);
void mergeEdges();

