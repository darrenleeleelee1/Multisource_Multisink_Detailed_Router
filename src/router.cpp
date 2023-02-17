#include <queue>
#include <cmath>
#include <iostream>
#include "router.hpp"
const std::vector<int> Router::x_orientation = {1, 0, -1, 0};
const std::vector<int> Router::y_orientation = {0, -1, 0, 1};
int maze_route_cost(Router *R, Coordinate3D sp, Coordinate3D ep){
    int cost = 0;
    if(sp.z != ep.z){
        cost += R->layout->via_cost;
    }
    cost += std::abs(sp.x - ep.x) * R->layout->horizontal_segment_cost + std::abs(sp.y - ep.y) * R->layout->vertical_segment_cost;
    return cost;
}

bool Router::outOfBound(Coordinate3D p){
    if(p.x < 0 || p.x > this->layout->width) return true;
    else if(p.y < 0 || p.y > this->layout->height) return true;
    else return false;
}
void Router::twoPinNetDecomposition(){
    for(auto &n : this->layout->netlist) {
        n.rmst_kruskal(this->layout->via_cost
            , this->layout->horizontal_segment_cost, this->layout->vertical_segment_cost);
    }
}
bool Router::pin2pin_maze_routing(Net *net, Coordinate3D source_node, Coordinate3D sink_node, int &reroute_status){
    bool success = true;
    Vertex *current;
    auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
    std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
    // ::: Initilize :::
    // Let the second point be the sink
    this->grid->setSinks(sink_node);
    // Set all vertex's distance to infinity
    this->grid->setDistanceInfinity();
    // Set the source's distance to zero
    this->grid->setDistanceZero(source_node);
    pq.push(this->grid->graph.at(source_node.x)
        .at(source_node.y).at(source_node.z));
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
            if(outOfBound(Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y+ this->y_orientation.at(i), i % 2})) continue;
            if(this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->isObstacle()
                && !(this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->is_sink)) continue;
            if(current->coordinate.z != (i % 2)){
                if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->isObstacle()
                    && !(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink)) continue;
            }
            if(current->distance + maze_route_cost(this, current->coordinate, Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y + this->y_orientation.at(i), i % 2})
                    < this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->distance){
                this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->prevertex = current;
                this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->distance 
                    = current->distance + maze_route_cost(this, current->coordinate, Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y + this->y_orientation.at(i), i % 2}); 
                pq.push(this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2));
            }
                
        }
    }
    // ::: Dijkstra :::
    // ::: Backtracking :::
    Path *tmp_path = new Path();
    net->paths.push_back(tmp_path);

    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
        if(current->coordinate == sink_node){
            // Pin location
            tmp_path->start_pin = current->coordinate;
        }
        else{
            // Via location, set z to negative
            tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }
    else{
        if(current->coordinate.x == sink_node.x && current->coordinate.y == sink_node.y
                && (current->coordinate.z + 1) % 2 == sink_node.z){
            // Pin location
            tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
        }
        else{
            // Via location, set z to negative
            tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }

    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->distance == 0){
        std::cout << "Failed: Net#" << net->id << " " << source_node.toString() << "-" << sink_node.toString() << " routing failed.\n";
        success = false; // 
    }
    else{
        Segment *tmp_seg = nullptr;
        while(current->prevertex != nullptr){
            current->obstacle = net->id;
            if(tmp_seg == nullptr){
                tmp_seg = new Segment();
                tmp_seg->z = current->coordinate.z;
                tmp_seg->x = current->coordinate.x;
                tmp_seg->y = current->coordinate.y;
            }
            if(tmp_seg->z != current->coordinate.z){
                this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->obstacle = net->id;
                if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
                    if(tmp_seg->z == 0){
                        tmp_seg->neighbor = current->coordinate.x;
                    }
                    else{
                        tmp_seg->neighbor = current->coordinate.y;
                    }
                    if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
                    tmp_seg = new Segment(); 
                }
                tmp_seg->z = current->coordinate.z;
                tmp_seg->x = current->coordinate.x;
                tmp_seg->y = current->coordinate.y;
            }
            current = current->prevertex;
        }
        if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
            if(tmp_seg->z == 0){
                tmp_seg->neighbor = current->coordinate.x;
            }
            else{
                tmp_seg->neighbor = current->coordinate.y;
            }
            if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
        }
    }

    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
        if(current->coordinate == source_node){
            // Pin location
            tmp_path->start_pin = current->coordinate;
        }
        else{
            // Via location, set z to negative
            tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }
    else{
        if(current->coordinate.x == source_node.x && current->coordinate.y == source_node.y
                && (current->coordinate.z + 1) % 2 == source_node.z){
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
    this->grid->resetSinks(sink_node);
    return success; // source_node to sink_node have paths
}
bool Router::tree2tree_maze_routing(Net *net, Coordinate3D source_node, Coordinate3D sink_node, int &reroute_status){
    bool success = true;
    Vertex *current;
    auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
    std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
    // ::: Initilize :::
    // Let the second point be the sink
    this->grid->setSinks(sink_node);
    for(auto &p : net->paths){
        if(p->start_pin == sink_node || p->end_pin == sink_node){
            for(auto &s : p->segments){
                this->grid->setSinks(*s);
            }
        }
    }
    // Set all vertex's distance to infinity
    this->grid->setDistanceInfinity();
    // Set the source's distance to zero
    this->grid->setDistanceZero(source_node);
    pq.push(this->grid->graph.at(source_node.x)
        .at(source_node.y).at(source_node.z));
    for(auto &p : net->paths){
        if(p->start_pin == source_node || p->end_pin == source_node){
            for(auto &s : p->segments){
                if(s->z == 0){
                    for(int i = std::min(s->x, s->neighbor); i <= std::max(s->x, s->neighbor); i++){
                        if(this->grid->graph.at(i).at(s->y).at(s->z)->distance != 0){
                            pq.push(this->grid->graph.at(i).at(s->y).at(s->z));
                        }
                    }
                }
                else{
                    for(int i = std::min(s->y, s->neighbor); i <= std::max(s->y, s->neighbor); i++){
                        if(this->grid->graph.at(s->x).at(i).at(s->z)->distance != 0){
                            pq.push(this->grid->graph.at(s->x).at(i).at(s->z));
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
            if(outOfBound(Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y+ this->y_orientation.at(i), i % 2})) continue;
            if(this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->isObstacle()
                && !(this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->is_sink)) continue;
            if(current->coordinate.z != (i % 2)){
                if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->isObstacle()
                    && !(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink)) continue;
            }
            if(current->distance + maze_route_cost(this, current->coordinate, Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y + this->y_orientation.at(i), i % 2})
                    < this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->distance){
                this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->prevertex = current;
                this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->distance 
                    = current->distance + maze_route_cost(this, current->coordinate, Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y + this->y_orientation.at(i), i % 2}); 
                pq.push(this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2));
            }
        }
    }
    // ::: Dijkstra :::
    // ::: Backtracking :::
    Path *tmp_path = new Path();
    net->paths.push_back(tmp_path);

    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
        if(current->coordinate == sink_node){
            // Pin location
            tmp_path->start_pin = current->coordinate;
        }
        else{
            // Via location, set z to negative
            tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }
    else{
        if(current->coordinate.x == sink_node.x && current->coordinate.y == sink_node.y
                && (current->coordinate.z + 1) % 2 == sink_node.z){
            // Pin location
            tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
        }
        else{
            // Via location, set z to negative
            tmp_path->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }

    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->distance == 0){
        if(!this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink){
            std::cout << "Failed: Net#" << net->id << " " << source_node.toString() << "-" << sink_node.toString() << " routing failed.\n";
            success = false;
            reroute_status = 1;

        }
    }
    else if(!this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink
            && !this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink){
        std::cout << "Failed: Net#" << net->id << " " << source_node.toString() << "-" << sink_node.toString() << " routing failed.\n";
        success = false;
        reroute_status = 2;
    }
    else{
        Segment *tmp_seg = nullptr;
        while(current->prevertex != nullptr){
            if(tmp_seg == nullptr){
                tmp_seg = new Segment();
                tmp_seg->z = current->coordinate.z;
                tmp_seg->x = current->coordinate.x;
                tmp_seg->y = current->coordinate.y;
            }
            if(tmp_seg->z != current->coordinate.z){
                this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->obstacle = true;
                if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
                    if(tmp_seg->z == 0){
                        tmp_seg->neighbor = current->coordinate.x;
                    }
                    else{
                        tmp_seg->neighbor = current->coordinate.y;
                    }
                    if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
                    tmp_seg = new Segment(); 
                }
                tmp_seg->z = current->coordinate.z;
                tmp_seg->x = current->coordinate.x;
                tmp_seg->y = current->coordinate.y;
            }
            current->obstacle = net->id;
            if(tmp_seg != nullptr) current->cur_segments.push_back(tmp_seg);
            current = current->prevertex;
        }
        current->obstacle = net->id;
        if(tmp_seg != nullptr) current->cur_segments.push_back(tmp_seg);

        if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
            if(tmp_seg->z == 0){
                tmp_seg->neighbor = current->coordinate.x;
            }
            else{
                tmp_seg->neighbor = current->coordinate.y;
            }
            if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
        }
    }
    
    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->distance == 0){
        if(current->coordinate == source_node){
            // Pin location
            tmp_path->end_pin = current->coordinate;
        }
        else{
            // Via location, set z to negative
            tmp_path->end_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }
    else{
        if(current->coordinate.x == source_node.x && current->coordinate.y == source_node.y
                && (current->coordinate.z + 1) % 2 == source_node.z){
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
    this->grid->resetSinks(sink_node);
    for(auto &p : net->paths){
        if(p->start_pin == sink_node || p->end_pin == sink_node){
            for(auto &s : p->segments){
                this->grid->resetSinks(*s);
            }
        }
    }
    return success;
}