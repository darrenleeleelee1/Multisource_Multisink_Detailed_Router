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
void Router::pin2pin_maze_routing(Net *net){
    for(auto &tpn : net->two_pins_net){
        Vertex *current;
        auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
        std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
        // ::: Initilize :::
        // Let the second point be the sink
        this->grid->setSinks(net->pins.at(tpn.second));
        // Set all vertex's distance to infinity
        this->grid->setDistanceInfinity();
        // Set the source's distance to zero
        this->grid->setDistanceZero(net->pins.at(tpn.first));
        pq.push(this->grid->graph.at(net->pins.at(tpn.first).x)
            .at(net->pins.at(tpn.first).y).at(net->pins.at(tpn.first).z));
        // Set all vertex's prevertex to nullptr
        this->grid->setPrevertexNull();
        // ::: Initilize :::
        // ::: Dijkstra :::
        while(!pq.empty()){
            current = pq.top(); pq.pop();
            
            if(current->is_sink || this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink){
                break;
            }
            // Enumerate 4 directions
            for(int i = 0; i < 4; i++){
                if(outOfBound(this, Coordinate3D{current->coordinate.x + x_orientation.at(i), current->coordinate.y+ y_orientation.at(i), i % 2})) continue;
                if(this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2)->is_obstacle
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
        Path *tmp_path = new Path();
        net->paths.push_back(tmp_path);

        if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
            if(current->coordinate == net->pins.at(tpn.second)){
                // Pin location
                tmp_path->start_pin = current->coordinate;
            }
            else{
                // Via location, set z to negative
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }
        else{
            if(current->coordinate.x == net->pins.at(tpn.second).x && current->coordinate.y == net->pins.at(tpn.second).y
                    && (current->coordinate.z + 1) % 2 == net->pins.at(tpn.second).z){
                // Pin location
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
            }
            else{
                // Via location, set z to negative
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }

        if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->distance == 0){
            std::cout << "Error: Net#" << net->id << " pin#" << tpn.first << "-pin#" << tpn.second << " routing failed.\n";
        }
        else{
            Segment *tmp_seg = nullptr;
            while(current->prevertex != nullptr){
                current->is_obstacle = true;
                if(tmp_seg == nullptr){
                    tmp_seg = new Segment();
                    tmp_seg->attribute = current->coordinate.z;
                    tmp_seg->x = current->coordinate.x;
                    tmp_seg->y = current->coordinate.y;
                }
                if(tmp_seg->attribute != current->coordinate.z){
                    this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_obstacle = true;
                    if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
                        if(tmp_seg->attribute == 0){
                            tmp_seg->neighbor = current->coordinate.x;
                        }
                        else{
                            tmp_seg->neighbor = current->coordinate.y;
                        }
                        if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
                        tmp_seg = new Segment(); 
                    }
                    tmp_seg->attribute = current->coordinate.z;
                    tmp_seg->x = current->coordinate.x;
                    tmp_seg->y = current->coordinate.y;
                }
                current = current->prevertex;
            }
            if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
                if(tmp_seg->attribute == 0){
                    tmp_seg->neighbor = current->coordinate.x;
                }
                else{
                    tmp_seg->neighbor = current->coordinate.y;
                }
                if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
            }
        }

        if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
            if(current->coordinate == net->pins.at(tpn.first)){
                // Pin location
                tmp_path->start_pin = current->coordinate;
            }
            else{
                // Via location, set z to negative
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }
        else{
            if(current->coordinate.x == net->pins.at(tpn.first).x && current->coordinate.y == net->pins.at(tpn.first).y
                    && (current->coordinate.z + 1) % 2 == net->pins.at(tpn.first).z){
                // Pin location
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
            }
            else{
                // Via location, set z to negative
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }
        // ::: Backtracking :::
        // Let the second point reset to not the sink
        this->grid->resetSinks(net->pins.at(tpn.second));
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
        for(auto &p : net->paths){
            if(p->start_pin == net->pins.at(tpn.second) || p->end_pin == net->pins.at(tpn.second)){
                for(auto &s : p->segments){
                    this->grid->setSinks(*s);
                }
            }
        }
        // Set all vertex's distance to infinity
        this->grid->setDistanceInfinity();
        // Set the source's distance to zero
        this->grid->setDistanceZero(net->pins.at(tpn.first));
        pq.push(this->grid->graph.at(net->pins.at(tpn.first).x)
            .at(net->pins.at(tpn.first).y).at(net->pins.at(tpn.first).z));
        for(auto &p : net->paths){
            if(p->start_pin == net->pins.at(tpn.first) || p->end_pin == net->pins.at(tpn.first)){
                for(auto &s : p->segments){
                    if(s->attribute == 0){
                        for(int i = std::min(s->x, s->neighbor); i <= std::max(s->x, s->neighbor); i++){
                            if(this->grid->graph.at(i).at(s->y).at(s->attribute)->distance != 0){
                                pq.push(this->grid->graph.at(i).at(s->y).at(s->attribute));
                            }
                        }
                    }
                    else{
                        for(int i = std::min(s->y, s->neighbor); i <= std::max(s->y, s->neighbor); i++){
                            if(this->grid->graph.at(s->x).at(i).at(s->attribute)->distance != 0){
                                pq.push(this->grid->graph.at(s->x).at(i).at(s->attribute));
                            }
                        }
                    }
                    this->grid->setDistanceZero(*s);
                }
            }
        }
        // Set all vertex's prevertex to nullptr
        this->grid->setPrevertexNull();
        // ::: Initilize :::
        // ::: Dijkstra :::
        while(!pq.empty()){
            current = pq.top(); pq.pop();
            
            if(current->is_sink || this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink){
                break;
            }
            // Enumerate 4 directions
            for(int i = 0; i < 4; i++){
                if(outOfBound(this, Coordinate3D{current->coordinate.x + x_orientation.at(i), current->coordinate.y+ y_orientation.at(i), i % 2})) continue;
                if(this->grid->graph.at(current->coordinate.x + x_orientation.at(i)).at(current->coordinate.y + y_orientation.at(i)).at(i % 2)->is_obstacle
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
        Path *tmp_path = new Path();
        net->paths.push_back(tmp_path);

        if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
            if(current->coordinate == net->pins.at(tpn.second)){
                // Pin location
                tmp_path->start_pin = current->coordinate;
            }
            else{
                // Via location, set z to negative
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }
        else{
            if(current->coordinate.x == net->pins.at(tpn.second).x && current->coordinate.y == net->pins.at(tpn.second).y
                    && (current->coordinate.z + 1) % 2 == net->pins.at(tpn.second).z){
                // Pin location
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
            }
            else{
                // Via location, set z to negative
                tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }

        if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->distance == 0){
            std::cout << "Error: Net#" << net->id << " pin#" << tpn.first << "-pin#" << tpn.second << " routing failed.\n";
        }
        else{
            Segment *tmp_seg = nullptr;
            while(current->prevertex != nullptr){
                current->is_obstacle = true;
                if(tmp_seg == nullptr){
                    tmp_seg = new Segment();
                    tmp_seg->attribute = current->coordinate.z;
                    tmp_seg->x = current->coordinate.x;
                    tmp_seg->y = current->coordinate.y;
                }
                if(tmp_seg->attribute != current->coordinate.z){
                    this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_obstacle = true;
                    if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
                        if(tmp_seg->attribute == 0){
                            tmp_seg->neighbor = current->coordinate.x;
                        }
                        else{
                            tmp_seg->neighbor = current->coordinate.y;
                        }
                        if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
                        tmp_seg = new Segment(); 
                    }
                    tmp_seg->attribute = current->coordinate.z;
                    tmp_seg->x = current->coordinate.x;
                    tmp_seg->y = current->coordinate.y;
                }
                current = current->prevertex;
            }
            if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
                if(tmp_seg->attribute == 0){
                    tmp_seg->neighbor = current->coordinate.x;
                }
                else{
                    tmp_seg->neighbor = current->coordinate.y;
                }
                if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
            }
        }

        if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->distance == 0){
            if(current->coordinate == net->pins.at(tpn.first)){
                // Pin location
                tmp_path->end_pin = current->coordinate;
            }
            else{
                // Via location, set z to negative
                tmp_path->end_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }
        else{
            if(current->coordinate.x == net->pins.at(tpn.first).x && current->coordinate.y == net->pins.at(tpn.first).y
                    && (current->coordinate.z + 1) % 2 == net->pins.at(tpn.first).z){
                // Pin location
                tmp_path->end_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
            }
            else{
                // Via location, set z to negative
                tmp_path->end_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
            }
        }
        // ::: Backtracking :::
        // Let the second point reset to not the sink
        this->grid->resetSinks(net->pins.at(tpn.second));
        for(auto &p : net->paths){
            if(p->start_pin == net->pins.at(tpn.second) || p->end_pin == net->pins.at(tpn.second)){
                for(auto &s : p->segments){
                    this->grid->resetSinks(*s);
                }
            }
        }

        // for(int j = this->grid->graph.at(0).size()-1; j >= 0; j--){
        //     for(unsigned i = 0; i < this->grid->graph.size(); i++){
        //         std::cout << (this->grid->graph.at(i).at(j).at(0)->is_obstacle) << " ";
        //     }
        //     std::cout << "\n";
        // }
        // std::cout << "\n";
        // for(int j = this->grid->graph.at(0).size()-1; j >= 0; j--){
        //     for(unsigned i = 0; i < this->grid->graph.size(); i++){
        //         std::cout << (this->grid->graph.at(i).at(j).at(1)->is_obstacle) << " ";
        //     }
        //     std::cout << "\n";
        // }
        // std::cout << "\n\n";
    }
}