#pragma once
#include <memory>
#include "layout.hpp"
#include "grid.hpp"
class Router
{
public:
    Layout *layout;
    Grid *grid;
    static const std::vector<int> x_orientation;
    static const std::vector<int> y_orientation;
    Router() {}
    Router(Layout *l) {
        this->layout = l;
        this->grid = new Grid(l->width, l->height);
        // number of net + 1 means obstacle
        for(auto o : l->obstacles) this->grid->setObstacles(l->netlist.size() + 1, o.start_point, o.end_point);
        for(auto n : l->netlist){
            for(auto p : n.pins) this->grid->setObstacles(n.id, p, p);
            // for(auto hs : n.horizontal_segments) this->grid->setObstacles(hs.start_point, hs.end_point);
            // for(auto vs : n.vertical_segments) this->grid->setObstacles(vs.start_point, vs.end_point);
        }
    }
    ~Router() {
        delete grid;
    }

    void main();
    void twoPinNetDecomposition();
    bool outOfBound(Coordinate3D p);
    bool pin2pin_maze_routing(Net *net, Coordinate3D source_node, Coordinate3D sink_node, int &reroute_status);
    bool tree2tree_maze_routing(Net *net, Subtree *source, Subtree *sink, int &reroute_status);
};

