#include <queue>
#include <cmath>
#include "router.hpp"
void Router::twoPinNetDecomposition(){
    for(auto &n : this->layout->netlist) {
        n.rmst_kruskal(this->layout->via_cost
            , this->layout->horizontal_segment_cost, this->layout->vertical_segment_cost);
    }
}
int maze_route_cost(Router *R, Coordinate3D sp, Coordinate3D ep){
    int cost = 0;
    if(sp.z != ep.z){
        cost += R->layout->via_cost;
    }
    cost += std::abs(sp.x - ep.x) * R->layout->horizontal_segment_cost + std::abs(sp.y - ep.y) * R->layout->vertical_segment_cost;
    return cost;
}
void Router::tree2tree_maze_routing(Net *net){
    // Let all the net pins as sink
    this->grid->setSinks(net->pins);
    
    for(auto &tpn : net->two_pins_net){
        Vertex *current;
        Vertex *prenode = this->grid->graph.at(net->pins.at(tpn.first).x)
            .at(net->pins.at(tpn.first).y).at(net->pins.at(tpn.first).z);
        auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
        std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
        // ::: Initilize :::
        // Let the source point not the sink
        this->grid->resetSinks(net->pins.at(tpn.first));
        this->grid->setDistanceInfinity();
        this->grid->setDistanceZero(net->pins.at(tpn.first));
        pq.push(this->grid->graph.at(net->pins.at(tpn.first).x)
            .at(net->pins.at(tpn.first).y).at(net->pins.at(tpn.first).z));
        this->grid->setPrevertexNull();
        for(auto &st : net->subtrees){
            if(st.pins.count(tpn.first)){
                for(auto &p : st.paths){
                    for(auto &s : p.segments){
                        this->grid->setDistanceZero(s);
                        if(s.attribute == 0){
                            for(int i = std::min(s.x, s.neighbor); i <= std::max(s.x, s.neighbor); i++){
                                pq.push(this->grid->graph.at(i).at(s.y).at(s.neighbor));
                            }
                        }
                        else{
                            for(int i = std::min(s.y, s.neighbor); i <= std::max(s.y, s.neighbor); i++){
                                pq.push(this->grid->graph.at(s.x).at(i).at(s.neighbor));
                            }
                        }
                    }
                }
            }
            else{
                for(auto &p : st.paths){
                    for(auto &s : p.segments){
                        this->grid->setSinks(s);
                    }
                }
            }
        }
        // ::: Initilize :::
        // ::: Dijkstra :::
        while(!pq.empty()){
            current = pq.top(); pq.pop();
            if(current->is_sink){
                break;
            }
            // Enumerate 4 directions
            for(int i = 0; i < 4; i++){
                switch (i)
                {
                // right
                case 0:
                    if(maze_route_cost(this, current->coordinate, Coordinate3D{current->coordinate.x, current->coordinate.y, current->coordinate.z}))
                    break;
                // bot
                case 1:
                    break;
                // left
                case 2:
                    break;
                // top
                case 3:
                    break;
                default:
                    break;
                }
            }
        }
        // ::: Dijkstra :::

        

    
        // Let the source point turn back to sink
        this->grid->setSinks(net->pins.at(tpn.first));
    }

    // reset all the net pins
    this->grid->resetSinks(net->pins);


}