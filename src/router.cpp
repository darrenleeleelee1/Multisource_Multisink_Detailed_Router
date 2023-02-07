#include <queue>
#include <cmath>
#include <iostream>
#include "router.hpp"
std::vector<int> x_orientation = {1, 0, -1, 0};
std::vector<int> y_orientation = {0, -1, 0, 1};
int maze_route_cost(Router *R, Coordinate3D sp, Coordinate3D ep){
    int cost = 0;
    if(sp.z != ep.z){
        cost += R->layout->via_cost;
    }
    cost += std::abs(sp.x - ep.x) * R->layout->horizontal_segment_cost + std::abs(sp.y - ep.y) * R->layout->vertical_segment_cost;
    return cost;
}
bool outOfBound(Router *R, Coordinate3D p){
    if(p.x < 0 || p.x > R->layout->width) return true;
    else if(p.y < 0 || p.y > R->layout->height) return true;
    else return false;
}
void Router::twoPinNetDecomposition(){
    for(auto &n : this->layout->netlist) {
        n.rmst_kruskal(this->layout->via_cost
            , this->layout->horizontal_segment_cost, this->layout->vertical_segment_cost);
    }
}
void Router::tree2tree_maze_routing(Net *net){
    for(auto &tpn : net->two_pins_net){
        Vertex *current;
        auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
        std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
        // ::: Initilize :::
        // Let the second point be the sink
        this->grid->setSinks(net->pins.at(tpn.second));
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
            
            if(current->is_sink || this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink){
                if(!current->is_sink) {
                    this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->prevertex = current;
                    current = this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2);
                }
                break;
            }
            // Enumerate 4 directions
            for(int i = 0; i < 4; i++){
                if(outOfBound(this, Coordinate3D{current->coordinate.x + x_orientation.at(i), current->coordinate.y+ y_orientation.at(i), i % 2})) continue;
                if(this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2)->obstacle
                    && !(this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2)->is_sink)) continue;
                if(current ->distance + maze_route_cost(this, current->coordinate, Coordinate3D{current->coordinate.x + x_orientation.at(i), current->coordinate.y + y_orientation.at(i), i % 2})
                        < this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2)->distance){
                    this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2)->prevertex = current;
                    this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2)->distance 
                        = current ->distance + maze_route_cost(this, current->coordinate, Coordinate3D{current->coordinate.x + x_orientation.at(i), current->coordinate.y + y_orientation.at(i), i % 2}); 
                    pq.push(this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2));
                }
                    
            }
        }
        // ::: Dijkstra :::
        // ::: Backtracking :::
        if(current->coordinate == this->grid->graph.at(net->pins.at(tpn.first).x).at(net->pins.at(tpn.first).y).at(net->pins.at(tpn.first).z)->coordinate){
            std::cout << "Error: Net#" << net->id << " pin#" << tpn.first << "-pin#" << tpn.second << " routing failed.\n";
        }
        else{
            Segment *tmp_s = nullptr;
            while(current->prevertex != nullptr){
                current->obstacle = true;
                if(tmp_s == nullptr){
                    tmp_s = new Segment();
                    tmp_s->attribute = current->coordinate.z;
                    tmp_s->x = current->coordinate.x;
                    tmp_s->y = current->coordinate.y;
                }
                if(tmp_s->attribute != current->coordinate.z){
                    net->vialist.emplace_back(current->coordinate.x, current->coordinate.y);
                    this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->obstacle = true;
                    if(tmp_s->x != current->coordinate.x || tmp_s->y != current->coordinate.y){
                        if(tmp_s->attribute == 0){
                            net->horizontal_segments.emplace_back(tmp_s->x, tmp_s->y, tmp_s->attribute
                                , current->coordinate.x, current->coordinate.y, tmp_s->attribute);
                        }
                        else{
                            net->vertical_segments.emplace_back(tmp_s->x, tmp_s->y, tmp_s->attribute
                                , current->coordinate.x, current->coordinate.y, tmp_s->attribute);
                        }
                    }
                    tmp_s->attribute = current->coordinate.z;
                    tmp_s->x = current->coordinate.x;
                    tmp_s->y = current->coordinate.y;
                }
                current = current->prevertex;
            }
            if(current->coordinate.z == 1){
                net->vialist.emplace_back(current->coordinate.x, current->coordinate.y);
            }
            if(tmp_s->attribute == 0){
                net->horizontal_segments.emplace_back(tmp_s->x, tmp_s->y, tmp_s->attribute
                    , current->coordinate.x, current->coordinate.y, tmp_s->attribute);
            }
            else{
                net->vertical_segments.emplace_back(tmp_s->x, tmp_s->y, tmp_s->attribute
                    , current->coordinate.x, current->coordinate.y, tmp_s->attribute);
            }
            delete tmp_s;
            // ::: Backtracking :::
        }
        // Let the second point reset to not the sink
        this->grid->resetSinks(net->pins.at(tpn.second));
    }
}