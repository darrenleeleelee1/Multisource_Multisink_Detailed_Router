#pragma once
#include <memory>
#include "layout.hpp"
#include "grid.hpp"
class Router
{
public:
    Layout *layout;
    Grid *grid;
    Router() {}
    Router(Layout *l) {
        this->layout = l;
        this->grid = new Grid(l->width, l->height);
        for(auto n : l->netlist){
            for(auto p : n.pins) this->grid->setObstacles(p, p);
            for(auto o : l->obstacles) this->grid->setObstacles(o.start_point, o.end_point);
            // for(auto hs : n.horizontal_segments) this->grid->setObstacles(hs.start_point, hs.end_point);
            // for(auto vs : n.vertical_segments) this->grid->setObstacles(vs.start_point, vs.end_point);
        }
    }
    ~Router() {
        delete grid;
    }

    void main();
    void twoPinNetDecomposition();
    bool pin2pin_maze_routing(Net *net, Coordinate3D source_node, Coordinate3D sink_node);
    bool tree2tree_maze_routing(Net *net, Coordinate3D source_node, Coordinate3D sink_node);
};
