#include "router.hpp"

void Router::twoPinNetDecomposition(){
    for(auto &n : this->layout->netlist) {
        n.rmst_kruskal(this->layout->via_cost
            , this->layout->horizontal_segment_cost, this->layout->vertical_segment_cost);
    }
}