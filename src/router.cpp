#include <queue>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <type_traits>
#include "router.hpp"
const std::vector<std::vector<Coordinate3D>> Router::move_orientation = {{Coordinate3D(1,0,0), Coordinate3D(-1,0,0), Coordinate3D(0,0,1)},
                            {Coordinate3D(0,1,0), Coordinate3D(0,-1,0), Coordinate3D(0,0,-1)}};
int mazeRouteCost(Router *R, Coordinate3D sp, Coordinate3D ep){
    int cost = 0;
    if(sp.z != ep.z){
        cost += R->layout->via_cost;
    }
    cost += std::abs(sp.x - ep.x) * R->layout->horizontal_segment_cost + std::abs(sp.y - ep.y) * R->layout->vertical_segment_cost;
    return cost;
}
void updateGridCurEdge(Grid *grid, Edge *split_candidate, Edge *new_edge, Segment *s){
    if(s->z == 0){
        for(int k = s->getX(); k <= s->getNeighbor(); k++){
            // Find the iterator for the element to remove
            auto &cur_edge = grid->graph.at(k).at(s->getY()).at(s->z)->cur_edges;
            auto it = std::find(cur_edge.begin(), cur_edge.end(), split_candidate);
            // If the element was found, remove it from the vector
            if (it != cur_edge.end()) {
                cur_edge.erase(it);
            }
            cur_edge.push_back(new_edge);
        }
    }
    else if(s->z == 1){
        for(int k = s->getY(); k <= s->getNeighbor(); k++){
            // Find the iterator for the element to remove
            auto &cur_edge = grid->graph.at(s->getX()).at(k).at(s->z)->cur_edges;
            auto it = std::find(cur_edge.begin(), cur_edge.end(), split_candidate);
            // If the element was found, remove it from the vector
            if (it != cur_edge.end()) {
                cur_edge.erase(it);
            }
            cur_edge.push_back(new_edge);
        }
    }
}
bool splitEdges(Grid *grid, Coordinate3D point, Edge *split_candidate, std::vector<Edge*> &updated_edges){
    for(unsigned i = 0; i < updated_edges.size(); i++){
        auto &p = updated_edges.at(i);
        if(p == split_candidate){
            Coordinate3D start_point = p->start_pin;
            Edge *new_edge = new Edge();
            bool complete_edge = false;;
            new_edge->start_pin = p->start_pin;
            std::vector<Segment*> remove_list;
            for(unsigned j = 0; j < p->segments.size(); j++){
                auto &s = p->segments.at(j);
                if(s->colinear(point)){
                    // Split segment
                    if(!(point == s->startPoint() || point == s->endPoint())){
                        Segment *new_segment = nullptr;
                        if(s->z == 0){
                            new_segment = new Segment(s->z, start_point.x, s->y, point.x);
                            if(s->neighbor == start_point.x) s->neighbor = point.x;
                            else s->x = point.x;
                        }
                        else{
                            new_segment = new Segment(s->z, s->x, start_point.y, point.y);
                            if(s->neighbor == start_point.y) s->neighbor = point.y;
                            else s->y = point.y;
                        }
                        new_edge->end_pin = point;
                        p->start_pin = point;
                        complete_edge = true;

                        updateGridCurEdge(grid, split_candidate, new_edge, new_segment);
                        grid->graph.at(point.x).at(point.y).at(point.z)->cur_edges.push_back(split_candidate);
                        if(new_segment != nullptr) new_edge->segments.push_back(new_segment);
                        
                    }
                    // Assign to old one
                    else if(point == start_point){
                        new_edge->end_pin = start_point;
                        p->start_pin = new_edge->end_pin;
                        complete_edge = true;
                    }
                    // Assign to new one
                    else{
                        updateGridCurEdge(grid, split_candidate, new_edge, s);
                        remove_list.push_back(s);
                        new_edge->segments.push_back(s);
                        start_point = (s->startPoint() == start_point ? Coordinate3D(s->endPoint().x, s->endPoint().y, (s->endPoint().z + 1) % 2) 
                                    : Coordinate3D(s->startPoint().x, s->startPoint().y, (s->startPoint().z + 1) % 2));
                        
                        new_edge->end_pin = start_point;
                        p->start_pin = new_edge->end_pin;
                        complete_edge = true;
                    }
                    break;
                }
                else{
                    updateGridCurEdge(grid, split_candidate, new_edge, s);
                    remove_list.push_back(s);
                    new_edge->segments.push_back(s);
                    start_point = (s->startPoint() == start_point ? Coordinate3D(s->endPoint().x, s->endPoint().y, (s->endPoint().z + 1) % 2) 
                                : Coordinate3D(s->startPoint().x, s->startPoint().y, (s->startPoint().z + 1) % 2));
                }
            }
            for (auto it = p->segments.begin(); it != p->segments.end(); ) {
                if (std::find(remove_list.begin(), remove_list.end(), *it) != remove_list.end()) {
                    // Remove the element
                    it = p->segments.erase(it);
                } else {
                    // Move to the next element
                    ++it;
                }
            }
            if(complete_edge) {
                updated_edges.push_back(new_edge);
                return true;
            }
            else{
                throw std::runtime_error("Error: split edge failed have unexpected error");
            }
        }
    }
    return false;
}
std::pair<int, int> ripUpEdges(Grid *grid, Edge *rip_up_candidate, Tree *updated_tree){
    // Remove the rip-up edge candidate
    for(auto s : rip_up_candidate->segments){
        if(s->z == 0){
            for(int i = s->getX(); i <= s->getNeighbor(); i++){
                auto &cur_edge = grid->graph.at(i).at(s->getY()).at(s->z)->cur_edges;
                cur_edge.erase(std::remove(cur_edge.begin(), cur_edge.end(), rip_up_candidate), cur_edge.end());
            }
        }
        else if(s->z == 1){
            for(int i = s->getY(); i <= s->getNeighbor(); i++){
                auto &cur_edge = grid->graph.at(s->getX()).at(i).at(s->z)->cur_edges;
                cur_edge.erase(std::remove(cur_edge.begin(), cur_edge.end(), rip_up_candidate), cur_edge.end());
            }
        }
        grid->resetObstacles(*s);
    }
    std::unordered_set<int> roots;
    for(unsigned i = 0; i < updated_tree->coordinate2index.size(); i++){
        int root = updated_tree->find(i);
        if(!roots.count(root)){
            roots.insert(root);
            auto &cur_edge = updated_tree->at(root)->edges;
            cur_edge.erase(std::remove(cur_edge.begin(), cur_edge.end(), rip_up_candidate), cur_edge.end());
        }
    }
    /* Merge Edges */
    // Check rip_up_candidate->start_pin
    unsigned edge_count = grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at(rip_up_candidate->start_pin.z)->cur_edges.size()
                + grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at((rip_up_candidate->start_pin.z + 1) % 2)->cur_edges.size();
    if(edge_count == 2 || edge_count == 3){
        std::vector<Edge*> merge_candidates;
        std::unordered_set<Edge*> remove_duplicate; remove_duplicate.insert(rip_up_candidate);
        for(auto p : grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at(rip_up_candidate->start_pin.z)->cur_edges){
            if(remove_duplicate.count(p)) continue;
            remove_duplicate.insert(p);
            merge_candidates.push_back(p);
        }
        for(auto p : grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at((rip_up_candidate->start_pin.z + 1) % 2)->cur_edges){
            if(remove_duplicate.count(p)) continue;
            remove_duplicate.insert(p);
            merge_candidates.push_back(p);
        }
        if(merge_candidates.size() != 2){
            throw std::runtime_error("Error: merge edge number exceed 2");
        }
        auto &first_edge = merge_candidates.at(0);
        auto &second_edge = merge_candidates.at(1);
        Segment *first_segment = nullptr, *second_segment = nullptr;
        for(auto fp : first_edge->segments){
            if(fp->startPoint() == rip_up_candidate->start_pin || fp->endPoint() == rip_up_candidate->start_pin){
                first_segment = fp;
            }
        }
        for(auto sp : second_edge->segments){
            if(sp->startPoint() == rip_up_candidate->start_pin || sp->endPoint() == rip_up_candidate->start_pin){
                second_segment = sp;
            }
        }
        if(first_segment->z == second_segment->z){
            first_segment->x = std::min(first_segment->getX(), second_segment->getX());
            first_segment->y = std::min(first_segment->getY(), second_segment->getY());
            first_segment->neighbor = std::max(first_segment->getNeighbor(), second_segment->getNeighbor());
            second_edge->segments.erase(std::remove(second_edge->segments.begin(), second_edge->segments.end(), second_segment), second_edge->segments.end());
            delete second_segment;
        }
        first_edge->segments.insert(first_edge->segments.end(), second_edge->segments.begin(), second_edge->segments.end());
        for(auto s : second_edge->segments){
            if(s->z == 0){
                for(int i = s->getX(); i <= s->getNeighbor(); i++){
                    auto &cur_edge = grid->graph.at(i).at(s->getY()).at(s->z)->cur_edges;
                    cur_edge.erase(std::remove(cur_edge.begin(), cur_edge.end(), second_edge), cur_edge.end());
                }
            }
            else if(s->z == 1){
                for(int i = s->getY(); i <= s->getNeighbor(); i++){
                    auto &cur_edge = grid->graph.at(s->getX()).at(i).at(s->z)->cur_edges;
                    cur_edge.erase(std::remove(cur_edge.begin(), cur_edge.end(), second_edge), cur_edge.end());
                }
            }
        }
        second_edge->segments.clear();
        if(first_edge->start_pin == second_edge->start_pin){

        }
        else if(first_edge->start_pin == second_edge->end_pin){
        }
        else if(first_edge->end_pin == second_edge->start_pin){
            first_edge->end_pin = second_edge->end_pin;
        }
        else if(first_edge->end_pin == second_edge->end_pin){

        }
        roots.clear();
        for(unsigned i = 0; i < updated_tree->coordinate2index.size(); i++){
			int root = updated_tree->find(i);
			if(!roots.count(root)){
				roots.insert(root);
                auto &cur_edge = updated_tree->at(root)->edges;
				cur_edge.erase(std::remove(cur_edge.begin(), cur_edge.end(), second_edge), cur_edge.end());
			}
		}
        delete second_edge;
    }
    if(grid->graph.at(rip_up_candidate->end_pin.x).at(rip_up_candidate->end_pin.y).at(rip_up_candidate->end_pin.z)->cur_edges.size() == 2){

    }

    updated_tree->reconstructTree();
    // Find the rip_up_candidate->start_pin at which tree
    int reroute_first_subtree = -1;
    roots.clear();
    for(unsigned i = 0; i < updated_tree->coordinate2index.size(); i++){
        int root = updated_tree->find(i);
        if(roots.count(root)) continue;
        roots.insert(root);
        if(updated_tree->at(root)->pinlist.count(rip_up_candidate->start_pin)){
            reroute_first_subtree = i;
            break;
        }
        for(auto e : updated_tree->at(root)->edges){
            for(auto s : e->segments){
                if(s->colinear(rip_up_candidate->start_pin)){
                    reroute_first_subtree = i;
                    break;
                }
            }
            if(reroute_first_subtree != -1) break;
        }
        if(reroute_first_subtree != -1) break;
    }
    // Find the rip_up_candidate->end_pin at which tree
    int reroute_second_subtree = -1;
    roots.clear();
    for(unsigned i = 0; i < updated_tree->coordinate2index.size(); i++){
        int root = updated_tree->find(i);
        if(roots.count(root)) continue;
        roots.insert(root);
        if(updated_tree->at(root)->pinlist.count(rip_up_candidate->end_pin)){
            reroute_second_subtree = i;
            break;
        }
        for(auto e : updated_tree->at(root)->edges){
            for(auto s : e->segments){
                if(s->colinear(rip_up_candidate->end_pin)){
                    reroute_second_subtree = i;
                    break;
                }
            }
            if(reroute_second_subtree != -1) break;
        }
        if(reroute_second_subtree != -1) break;
    }
    if(reroute_first_subtree == -1 || reroute_second_subtree == -1){
        throw std::runtime_error("Not find candidate subtree for reroute");
    }

    delete rip_up_candidate;
    return std::make_pair(reroute_first_subtree, reroute_second_subtree);
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
/*
bool Router::pin2pin_maze_routing(Net *net, Coordinate3D source_node, Coordinate3D sink_node, int &reroute_status){
    bool success = true;
    Vertex *current;
    auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
    std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
    // ::: Initilize :::
    // Let the second point be the sink
    for(auto &p : net->subtrees.pinlist.at(net->subtree.find(net->coordinate2index.at(sink_node)))){
        this->grid->setSinks(p);
    }
    // Set all vertex's distance to infinity
    this->grid->setDistanceInfinity();
    // Set the source's distance to zero
    for(auto &p : net->subtrees.pinlist.at(net->subtree.find(net->coordinate2index.at(source_node)))){
        this->grid->setDistanceZero(source_node);
    }
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
            if(current->distance + mazeRouteCost(this, current->coordinate, Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y + this->y_orientation.at(i), i % 2})
                    < this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->distance){
                this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->prevertex = current;
                this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2)->distance 
                    = current->distance + mazeRouteCost(this, current->coordinate, Coordinate3D{current->coordinate.x + this->x_orientation.at(i), current->coordinate.y + this->y_orientation.at(i), i % 2}); 
                pq.push(this->grid->graph.at(current->coordinate.x + this->x_orientation.at(i)).at(current->coordinate.y + this->y_orientation.at(i)).at(i % 2));
            }
                
        }
    }
    // ::: Dijkstra :::
    // ::: Backtracking :::
    Edge *tmp_edge = new Edge();
    net->edges.push_back(tmp_edge);

    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
        if(current->coordinate == sink_node){
            // Pin location
            tmp_edge->start_pin = current->coordinate;
        }
        else{
            // Via location, set z to negative
            tmp_edge->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }
    else{
        if(current->coordinate.x == sink_node.x && current->coordinate.y == sink_node.y
                && (current->coordinate.z + 1) % 2 == sink_node.z){
            // Pin location
            tmp_edge->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
        }
        else{
            // Via location, set z to negative
            tmp_edge->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
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
                    if(tmp_seg != nullptr) tmp_edge->segments.push_back(tmp_seg);
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
            if(tmp_seg != nullptr) tmp_edge->segments.push_back(tmp_seg);
        }
    }

    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
        if(current->coordinate == source_node){
            // Pin location
            tmp_edge->start_pin = current->coordinate;
        }
        else{
            // Via location, set z to negative
            tmp_edge->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }
    else{
        if(current->coordinate.x == source_node.x && current->coordinate.y == source_node.y
                && (current->coordinate.z + 1) % 2 == source_node.z){
            // Pin location
            tmp_edge->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, (current->coordinate.z + 1) % 2);
        }
        else{
            // Via location, set z to negative
            tmp_edge->start_pin = Coordinate3D(current->coordinate.x, current->coordinate.y, -1);
        }
    }
    // ::: Backtracking :::
    // Let the second point reset to not the sink
    this->grid->resetSinks(sink_node);
    return success; // source_node to sink_node have edges
}
*/
bool Router::tree2tree_maze_routing(Net *net, Subtree *source, Subtree *sink, int &reroute_status){
    /* Declaring */
    bool success = true;
    Vertex *current;
    auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
    std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
    /* Initialize the soruce and sink verteices */
    // Let the second point be the sink
    this->grid->setSinks(std::vector<Coordinate3D>(sink->pinlist.begin(), sink->pinlist.end()));
    for(auto &p : sink->edges){
        for(auto &s : p->segments){
            this->grid->setSinks(*s);
        }
    }
    // Set all vertex's distance to infinity
    this->grid->setDistanceInfinity();
    // Set the source's distance to zero
    this->grid->setDistanceZero(std::vector<Coordinate3D>(source->pinlist.begin(), source->pinlist.end()));
    for(auto p : source->pinlist){
        pq.push(this->grid->graph.at(p.x)
            .at(p.y).at(p.z));
    }
    for(auto &p : source->edges){
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
    // Set all vertex's prevertex to nullptr
    this->grid->setPrevertexNull();
    /* 
     * Dijkstra's algorithm for finding the shortest edge in tree to tree.
     * The algorithm starts from the source tree and explores all reachable vertices,
     * until it reaches the sink or there are no more vertices left to explore.
     */
    while(!pq.empty()){
        current = pq.top(); pq.pop();
        if(current->is_sink){
            break;
        }
        // Enumerate 4 directions
        int cur_z = current->coordinate.z;
        for(unsigned i = 0; i < move_orientation.at(cur_z).size(); i++){
            if(outOfBound(Coordinate3D{current->coordinate.x + move_orientation.at(cur_z).at(i).x, current->coordinate.y + move_orientation.at(cur_z).at(i).y, cur_z + move_orientation.at(cur_z).at(i).z})) continue;
            if(this->grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->isObstacle()
            && !this->grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->is_sink) continue;
            if(current->distance + mazeRouteCost(this, current->coordinate, Coordinate3D{current->coordinate.x + move_orientation.at(cur_z).at(i).x, current->coordinate.y + move_orientation.at(cur_z).at(i).y, cur_z + move_orientation.at(cur_z).at(i).z})
                    < this->grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->distance){
                this->grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->prevertex = current;
                this->grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->distance 
                    = current->distance + mazeRouteCost(this, current->coordinate, Coordinate3D{current->coordinate.x + move_orientation.at(cur_z).at(i).x, current->coordinate.y + move_orientation.at(cur_z).at(i).y, cur_z + move_orientation.at(cur_z).at(i).z}); 
                pq.push(this->grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z));
            }
        }
    }
    /* Backtracking */
    // Failed tree2tree routing
    if(this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->distance == 0){
        if(!this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink){
            std::cout << "Failed: Net#" << net->id << " " << source->showPins() << "- " << sink->showPins() << " routing failed\n";
            success = false;
            reroute_status = 1;

        }
    }
    else if(!this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink
            && !this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->is_sink){
        std::cout << "Failed: Net#" << net->id << " " << source->showPins() << "- " << sink->showPins() << " routing failed\n";
        success = false;
        reroute_status = 2;
    }
    // Tree2tree routing success
    else{
        Edge *tmp_edge = new Edge();
        tmp_edge->start_pin = current->coordinate;

        source->edges.push_back(tmp_edge);

        Segment *tmp_seg = nullptr;
        while(current->prevertex != nullptr){
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
                    if(tmp_seg != nullptr) tmp_edge->segments.push_back(tmp_seg);
                    tmp_seg = new Segment(); 
                }
                tmp_seg->z = current->coordinate.z;
                tmp_seg->x = current->coordinate.x;
                tmp_seg->y = current->coordinate.y;
            }
            current->obstacle = net->id;
            if(tmp_edge != nullptr) current->cur_edges.push_back(tmp_edge);
            current = current->prevertex;
        }

        current->obstacle = net->id;
        if(tmp_edge != nullptr) current->cur_edges.push_back(tmp_edge);

        if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
            if(tmp_seg->z == 0){
                tmp_seg->neighbor = current->coordinate.x;
            }
            else{
                tmp_seg->neighbor = current->coordinate.y;
            }
            if(tmp_seg != nullptr) tmp_edge->segments.push_back(tmp_seg);
        }

        tmp_edge->end_pin = current->coordinate;
        /* Split Edge */
        unsigned edge_count = this->grid->graph.at(tmp_edge->start_pin.x).at(tmp_edge->start_pin.y).at(tmp_edge->start_pin.z)->cur_edges.size()
                        + this->grid->graph.at(tmp_edge->start_pin.x).at(tmp_edge->start_pin.y).at((tmp_edge->start_pin.z + 1) % 2)->cur_edges.size();
        if(edge_count == 2 || edge_count == 3){
            if(!source->pinlist.count(tmp_edge->start_pin) && !source->pinlist.count(Coordinate3D{tmp_edge->start_pin.x, tmp_edge->start_pin.y, (tmp_edge->start_pin.z + 1) % 2})){
                if(!sink->pinlist.count(tmp_edge->start_pin) && !sink->pinlist.count(Coordinate3D{tmp_edge->start_pin.x, tmp_edge->start_pin.y, (tmp_edge->start_pin.z + 1) % 2})){
                    std::unordered_set<Edge*> splited; splited.insert(tmp_edge);
                    bool split = false;
                    for(auto &split_candidate_edge : this->grid->graph.at(tmp_edge->start_pin.x).at(tmp_edge->start_pin.y).at(tmp_edge->start_pin.z)->cur_edges){
                        if(splited.count(split_candidate_edge)) continue;
                        splited.insert(split_candidate_edge);
                        // Check source->edges contain the tmp_edge->start_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->start_pin, split_candidate_edge, source->edges);
                        // Check sink->edges contain the tmp_edge->start_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->start_pin, split_candidate_edge, sink->edges);
                    }
                    for(auto &split_candidate_edge : this->grid->graph.at(tmp_edge->start_pin.x).at(tmp_edge->start_pin.y).at((tmp_edge->start_pin.z + 1) % 2)->cur_edges){
                        if(splited.count(split_candidate_edge)) continue;
                        splited.insert(split_candidate_edge);
                        // Check source->edges contain the tmp_edge->start_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->start_pin, split_candidate_edge, source->edges);
                        // Check sink->edges contain the tmp_edge->start_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->start_pin, split_candidate_edge, sink->edges);
                    }
                }
            }
            
        }
        edge_count = this->grid->graph.at(tmp_edge->end_pin.x).at(tmp_edge->end_pin.y).at(tmp_edge->end_pin.z)->cur_edges.size()
                + this->grid->graph.at(tmp_edge->end_pin.x).at(tmp_edge->end_pin.y).at((tmp_edge->end_pin.z + 1) % 2)->cur_edges.size();
        if(edge_count == 2 || edge_count == 3){
            if(!source->pinlist.count(tmp_edge->end_pin) && !source->pinlist.count(Coordinate3D{tmp_edge->end_pin.x, tmp_edge->end_pin.y, (tmp_edge->end_pin.z + 1) % 2})){
                if(!sink->pinlist.count(tmp_edge->end_pin) && !sink->pinlist.count(Coordinate3D{tmp_edge->end_pin.x, tmp_edge->end_pin.y, (tmp_edge->end_pin.z + 1) % 2})){
                    std::unordered_set<Edge*> splited; splited.insert(tmp_edge);
                    bool split = false;
                    for(auto &split_candidate_edge :  this->grid->graph.at(tmp_edge->end_pin.x).at(tmp_edge->end_pin.y).at(tmp_edge->end_pin.z)->cur_edges){
                        if(splited.count(split_candidate_edge)) continue;
                        // Then check source->edges contain the tmp_edge->end_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->end_pin, split_candidate_edge, source->edges);
                        // Then check sink->edges contain the tmp_edge->end_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->end_pin, split_candidate_edge, sink->edges);
                    }
                    for(auto &split_candidate_edge :  this->grid->graph.at(tmp_edge->end_pin.x).at(tmp_edge->end_pin.y).at((tmp_edge->end_pin.z + 1) % 2)->cur_edges){
                        if(splited.count(split_candidate_edge)) continue;
                        // Then check source->edges contain the tmp_edge->end_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->end_pin, split_candidate_edge, source->edges);
                        // Then check sink->edges contain the tmp_edge->end_pin
                        if(!split) split = splitEdges(this->grid, tmp_edge->end_pin, split_candidate_edge, sink->edges);
                    }
                }
            }
        }
    }
    /* Post job cleanup */
    // Let the second point reset to not the sink
    this->grid->resetSinks(std::vector<Coordinate3D>(sink->pinlist.begin(), sink->pinlist.end()));
    for(auto &p : sink->edges){
        for(auto &s : p->segments){
            this->grid->resetSinks(*s);
        }
    }
    return success;
}