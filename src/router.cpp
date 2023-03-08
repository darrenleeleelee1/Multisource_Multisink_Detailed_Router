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
/* Insert the path from the grid */
void insertPathsToGrid(Grid *grid, Path *insert_candidate, int net_id){
    bool find = false;
    // Remove an `Path` including all the segments and the start_pin and end_pin location
    for(auto s : insert_candidate->segments){
        if(s->z == 0){
            for(int i = s->getX(); i <= s->getNeighbor(); i++){
                auto &cur_path = grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths;
                find = false;
                for(auto e : cur_path){
                    if(e == insert_candidate) find = true;
                    if(find) break;
                }
                if(!find) cur_path.push_back(insert_candidate);
            }
        }
        else if(s->z == 1){
            for(int i = s->getY(); i <= s->getNeighbor(); i++){
                auto &cur_path = grid->graph.at(s->getX()).at(i).at(s->z)->cur_paths;
                find = false;
                for(auto e : cur_path){
                    if(e == insert_candidate) find = true;
                    if(find) break;
                }
                if(!find) cur_path.push_back(insert_candidate);
            }
        }
        grid->setObstacles(net_id, *s);
    }
    // Remove start_pin
    auto sp = insert_candidate->start_pin;
    auto &t = grid->graph.at(sp.x).at(sp.y).at(sp.z)->cur_paths;
    find = false;
    for(auto e : t){
        if(e == insert_candidate) find = true;
        if(find) break;
    }
    if(!find) t.push_back(insert_candidate);
    grid->setObstacles(net_id, sp);
    // Remove end_pin
    auto ep = insert_candidate->end_pin;
    auto &k = grid->graph.at(ep.x).at(ep.y).at(ep.z)->cur_paths;
    find = false;
    for(auto e : k){
        if(e == insert_candidate) find = true;
        if(find) break;
    }
    if(!find) k.push_back(insert_candidate);
    grid->setObstacles(net_id, ep);
}
/* Remove the path from the grid */
void removePathsFromGrid(Grid *grid, Path *remove_candidate){
    // Remove an `Path` including all the segments and the start_pin and end_pin location
    for(auto s : remove_candidate->segments){
        if(s->z == 0){
            for(int i = s->getX(); i <= s->getNeighbor(); i++){
                auto &cur_path = grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths;
                cur_path.erase(std::remove(cur_path.begin(), cur_path.end(), remove_candidate), cur_path.end());
                if(cur_path.size() == 0) grid->resetObstacles(Coordinate3D{i, s->getY(), s->z});
            }
        }
        else if(s->z == 1){
            for(int i = s->getY(); i <= s->getNeighbor(); i++){
                auto &cur_path = grid->graph.at(s->getX()).at(i).at(s->z)->cur_paths;
                cur_path.erase(std::remove(cur_path.begin(), cur_path.end(), remove_candidate), cur_path.end());
                if(cur_path.size() == 0) grid->resetObstacles(Coordinate3D{s->getX(), i, s->z});
            }
        }
    }
    // Remove start_pin
    auto sp = remove_candidate->start_pin;
    auto &t = grid->graph.at(sp.x).at(sp.y).at(sp.z)->cur_paths;
    t.erase(std::remove(t.begin(), t.end(), remove_candidate), t.end());
    if(t.size() == 0) grid->resetObstacles(sp);
    // Remove end_pin
    auto ep = remove_candidate->end_pin;
    auto &k = grid->graph.at(ep.x).at(ep.y).at(ep.z)->cur_paths;
    k.erase(std::remove(k.begin(), k.end(), remove_candidate), k.end());
    if(k.size() == 0) grid->resetObstacles(ep);
}
/* Remove the remove_candidate from the grid along the remove_path_locus */
void removePathsFromGrid(Grid *grid, Path *remove_path_locus, Path *remove_candidate){
    // Remove an `Path` including all the segments and the start_pin and end_pin location
    for(auto s : remove_path_locus->segments){
        if(s->z == 0){
            for(int i = s->getX(); i <= s->getNeighbor(); i++){
                auto &cur_path = grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths;
                cur_path.erase(std::remove(cur_path.begin(), cur_path.end(), remove_candidate), cur_path.end());
                if(cur_path.size() == 0) grid->resetObstacles(Coordinate3D{i, s->getY(), s->z});
            }
        }
        else if(s->z == 1){
            for(int i = s->getY(); i <= s->getNeighbor(); i++){
                auto &cur_path = grid->graph.at(s->getX()).at(i).at(s->z)->cur_paths;
                cur_path.erase(std::remove(cur_path.begin(), cur_path.end(), remove_candidate), cur_path.end());
                if(cur_path.size() == 0) grid->resetObstacles(Coordinate3D{s->getX(), i, s->z});
            }
        }
    }
    // Remove start_pin
    auto sp = remove_path_locus->start_pin;
    auto &t = grid->graph.at(sp.x).at(sp.y).at(sp.z)->cur_paths;
    t.erase(std::remove(t.begin(), t.end(), remove_candidate), t.end());
    if(t.size() == 0) grid->resetObstacles(sp);
    // Remove end_pin
    auto ep = remove_path_locus->end_pin;
    auto &k = grid->graph.at(ep.x).at(ep.y).at(ep.z)->cur_paths;
    k.erase(std::remove(k.begin(), k.end(), remove_candidate), k.end());
    if(k.size() == 0) grid->resetObstacles(ep);
}
/* When creating a stiener node on a path, the path need to be split*/
bool splitPaths(Grid *grid, Coordinate3D point, Path *split_candidate, std::vector<Path*> &updated_paths, int net_id){
    for(unsigned i = 0; i < updated_paths.size(); i++){
        auto &p = updated_paths.at(i);
        if(p == split_candidate){
            Coordinate3D start_point = p->start_pin;
            Path *new_path = new Path();
            bool complete_path = false;;
            new_path->start_pin = p->start_pin;
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
                        new_path->end_pin = point;
                        p->start_pin = point;
                        if(new_segment != nullptr) new_path->segments.push_back(new_segment);
                        complete_path = true;
                    }
                    // Current segment belongs to old one
                    else if(point == start_point){
                        new_path->end_pin = start_point;
                        p->start_pin = new_path->end_pin;
                        complete_path = true;
                    }
                    // Current segment belongs to  new one, assign to new path
                    else{
                        remove_list.push_back(s);
                        new_path->segments.push_back(s);
                        start_point = ((s->startPoint().x == start_point.x && s->startPoint().y == start_point.y) ? Coordinate3D(s->endPoint().x, s->endPoint().y, (s->endPoint().z + 1) % 2) 
                                    : Coordinate3D(s->startPoint().x, s->startPoint().y, (s->startPoint().z + 1) % 2));
                        new_path->end_pin = start_point;
                        p->start_pin = new_path->end_pin;
                        complete_path = true;
                    }
                    break;
                }
                else{
                    remove_list.push_back(s);
                    new_path->segments.push_back(s);
                    start_point = ((s->startPoint().x == start_point.x && s->startPoint().y == start_point.y) ? Coordinate3D(s->endPoint().x, s->endPoint().y, (s->endPoint().z + 1) % 2) 
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
            if(complete_path) {
                insertPathsToGrid(grid, new_path, net_id);
                removePathsFromGrid(grid, new_path, p);
                insertPathsToGrid(grid, p, net_id);
                updated_paths.push_back(new_path);
                return true;
            }
            else{
                throw std::runtime_error("Error: split path failed have unexpected error");
            }
        }
    }
    return false;
}
/*
 * Rip up the `rip_up_candidate` and updated the `updated_tree`
 * Also updated the path that It was been merge
 */
std::pair<int, int> ripUpPaths(Grid *grid, Path *rip_up_candidate, Tree *updated_tree){
    int net_id = grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y)
                .at(rip_up_candidate->start_pin.z)->obstacle;
    if(net_id == -1){
        throw std::runtime_error("Net id == -1");
    }
    // Remove the rip-up path candidate
    removePathsFromGrid(grid, rip_up_candidate);

    std::unordered_set<int> roots;
    for(unsigned i = 0; i < updated_tree->coordinate2index.size(); i++){
        int root = updated_tree->find(i);
        if(!roots.count(root)){
            roots.insert(root);
            auto &cur_path = updated_tree->at(root)->paths;
            cur_path.erase(std::remove(cur_path.begin(), cur_path.end(), rip_up_candidate), cur_path.end());
        }
    }
    /* Merge Paths */
    // Check rip_up_candidate->start_pin
    std::unordered_set<Path*> path_count;
    path_count.insert(grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at(rip_up_candidate->start_pin.z)->cur_paths.begin()
                , grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at(rip_up_candidate->start_pin.z)->cur_paths.end());
    path_count.insert(grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at((rip_up_candidate->start_pin.z + 1) % 2)->cur_paths.begin()
                , grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at((rip_up_candidate->start_pin.z + 1) % 2)->cur_paths.end());
    if(!updated_tree->pinset.count(Coordinate3D{rip_up_candidate->start_pin.x, rip_up_candidate->start_pin.y, (rip_up_candidate->start_pin.z + 1) % 2}) 
        && !updated_tree->pinset.count(rip_up_candidate->start_pin) && (path_count.size() == 2)){
        std::vector<Path*> merge_candidates;
        std::unordered_set<Path*> remove_duplicate; remove_duplicate.insert(rip_up_candidate);
        for(auto p : grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at(rip_up_candidate->start_pin.z)->cur_paths){
            if(remove_duplicate.count(p)) continue;
            remove_duplicate.insert(p);
            merge_candidates.push_back(p);
        }
        for(auto p : grid->graph.at(rip_up_candidate->start_pin.x).at(rip_up_candidate->start_pin.y).at((rip_up_candidate->start_pin.z + 1) % 2)->cur_paths){
            if(remove_duplicate.count(p)) continue;
            remove_duplicate.insert(p);
            merge_candidates.push_back(p);
        }
        if(merge_candidates.size() != 2){
            throw std::runtime_error("Error: merge path number != 2");
        }
        auto &first_path = merge_candidates.at(0);
        auto &second_path = merge_candidates.at(1);
        Segment *first_segment = nullptr, *second_segment = nullptr;
        for(auto fp : first_path->segments){
            // Find the conjunction point of two segments
            if(Coordinate2D{fp->startPoint()} == Coordinate2D{rip_up_candidate->start_pin} 
                || Coordinate2D{fp->endPoint()} == Coordinate2D{rip_up_candidate->start_pin}){
                first_segment = fp;
            }
        }
        for(auto sp : second_path->segments){
            // Find the conjunction point of two segments
            if(Coordinate2D{sp->startPoint()} == Coordinate2D{rip_up_candidate->start_pin} 
                || Coordinate2D{sp->endPoint()} == Coordinate2D{rip_up_candidate->start_pin}){
                second_segment = sp;
            }
        }

        removePathsFromGrid(grid, first_path);
        removePathsFromGrid(grid, second_path);

        if(first_segment != nullptr && second_segment != nullptr && (first_segment->z == second_segment->z)){
            first_segment->x = std::min(first_segment->getX(), second_segment->getX());
            first_segment->y = std::min(first_segment->getY(), second_segment->getY());
            first_segment->neighbor = std::max(first_segment->getNeighbor(), second_segment->getNeighbor());
            second_path->segments.erase(std::remove(second_path->segments.begin(), second_path->segments.end(), second_segment), second_path->segments.end());
            delete second_segment;
        }
        for(auto s : second_path->segments){
            first_path->segments.push_back(new Segment(*s));
        }
        
        // Find the conjunction point and merge them to first_path
        if(Coordinate2D{first_path->start_pin} == Coordinate2D{second_path->start_pin}){
            first_path->start_pin = second_path->end_pin;
        }
        else if(Coordinate2D{first_path->start_pin} == Coordinate2D{second_path->end_pin}){
            first_path->start_pin = second_path->start_pin;
        }
        else if(Coordinate2D{first_path->end_pin} == Coordinate2D{second_path->start_pin}){
            first_path->end_pin = second_path->end_pin;
        }
        else if(Coordinate2D{first_path->end_pin} == Coordinate2D{second_path->end_pin}){
            first_path->end_pin = second_path->start_pin;
        }

        roots.clear();
        for(unsigned i = 0; i < updated_tree->coordinate2index.size(); i++){
			int root = updated_tree->find(i);
			if(!roots.count(root)){
				roots.insert(root);
                auto &cur_path = updated_tree->at(root)->paths;
				cur_path.erase(std::remove(cur_path.begin(), cur_path.end(), second_path), cur_path.end());
			}
		}
        if(second_path != nullptr) delete second_path;
    }
    // Check rip_up_candidate->end_pin
    path_count.clear();
    path_count.insert(grid->graph.at(rip_up_candidate->end_pin.x).at(rip_up_candidate->end_pin.y).at(rip_up_candidate->end_pin.z)->cur_paths.begin()
                , grid->graph.at(rip_up_candidate->end_pin.x).at(rip_up_candidate->end_pin.y).at(rip_up_candidate->end_pin.z)->cur_paths.end());
    path_count.insert(grid->graph.at(rip_up_candidate->end_pin.x).at(rip_up_candidate->end_pin.y).at((rip_up_candidate->end_pin.z + 1) % 2)->cur_paths.begin()
                , grid->graph.at(rip_up_candidate->end_pin.x).at(rip_up_candidate->end_pin.y).at((rip_up_candidate->end_pin.z + 1) % 2)->cur_paths.end());
    
    if(!updated_tree->pinset.count(Coordinate3D{rip_up_candidate->end_pin.x, rip_up_candidate->end_pin.y, (rip_up_candidate->end_pin.z + 1) % 2}) 
        && !updated_tree->pinset.count(rip_up_candidate->end_pin) && (path_count.size() == 2)){
        std::vector<Path*> merge_candidates;
        std::unordered_set<Path*> remove_duplicate; remove_duplicate.insert(rip_up_candidate);
        for(auto p : grid->graph.at(rip_up_candidate->end_pin.x).at(rip_up_candidate->end_pin.y).at(rip_up_candidate->end_pin.z)->cur_paths){
            if(remove_duplicate.count(p)) continue;
            remove_duplicate.insert(p);
            merge_candidates.push_back(p);
        }
        for(auto p : grid->graph.at(rip_up_candidate->end_pin.x).at(rip_up_candidate->end_pin.y).at((rip_up_candidate->end_pin.z + 1) % 2)->cur_paths){
            if(remove_duplicate.count(p)) continue;
            remove_duplicate.insert(p);
            merge_candidates.push_back(p);
        }
        if(merge_candidates.size() != 2){
            throw std::runtime_error("Error: merge path number exceed 2");
        }
        auto &first_path = merge_candidates.at(0);
        auto &second_path = merge_candidates.at(1);
        Segment *first_segment = nullptr, *second_segment = nullptr;
        for(auto fp : first_path->segments){
            // Find the conjunction point of two segments
            if(Coordinate2D{fp->startPoint()} == Coordinate2D{rip_up_candidate->end_pin} 
                || Coordinate2D{fp->endPoint()} == Coordinate2D{rip_up_candidate->end_pin}){
                first_segment = fp;
            }
        }
        for(auto sp : second_path->segments){
            // Find the conjunction point of two segments
            if(Coordinate2D{sp->startPoint()} == Coordinate2D{rip_up_candidate->end_pin} 
                || Coordinate2D{sp->endPoint()} == Coordinate2D{rip_up_candidate->end_pin}){
                second_segment = sp;
            }
        }

        removePathsFromGrid(grid, first_path);
        removePathsFromGrid(grid, second_path);

        if(first_segment != nullptr && second_segment != nullptr && (first_segment->z == second_segment->z)){
            first_segment->x = std::min(first_segment->getX(), second_segment->getX());
            first_segment->y = std::min(first_segment->getY(), second_segment->getY());
            first_segment->neighbor = std::max(first_segment->getNeighbor(), second_segment->getNeighbor());
            second_path->segments.erase(std::remove(second_path->segments.begin(), second_path->segments.end(), second_segment), second_path->segments.end());
            delete second_segment;
        }
        for(auto s : second_path->segments){
            first_path->segments.push_back(new Segment(*s));
        }

        // Find the conjunction point and merge them to first_path
        if(Coordinate2D{first_path->start_pin} == Coordinate2D{second_path->start_pin}){
            first_path->start_pin = second_path->end_pin;
        }
        else if(Coordinate2D{first_path->start_pin} == Coordinate2D{second_path->end_pin}){
            first_path->start_pin = second_path->start_pin;
        }
        else if(Coordinate2D{first_path->end_pin} == Coordinate2D{second_path->start_pin}){
            first_path->end_pin = second_path->end_pin;
        }
        else if(Coordinate2D{first_path->end_pin} == Coordinate2D{second_path->end_pin}){
            first_path->end_pin = second_path->start_pin;
        }
        else{
            throw std::runtime_error("Fail merge first_path and second_path");
        }

        roots.clear();
        for(unsigned i = 0; i < updated_tree->coordinate2index.size(); i++){
			int root = updated_tree->find(i);
			if(!roots.count(root)){
				roots.insert(root);
                auto &cur_path = updated_tree->at(root)->paths;
				cur_path.erase(std::remove(cur_path.begin(), cur_path.end(), second_path), cur_path.end());
			}
		}

        if(second_path != nullptr) delete second_path;
    }

    
    const auto &[old_paths, new_paths] = updated_tree->reconstructTree_phase1();
    for(auto e : old_paths){
        removePathsFromGrid(grid, e);
    }
    for(auto e : new_paths){
        insertPathsToGrid(grid, e, net_id);
    }
    // Some pins may be clear out, need to insert back
    for(auto p : updated_tree->pinset){
        grid->setObstacles(net_id, p);
    }
    updated_tree->reconstructTree_phase2(new_paths);

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
        for(auto e : updated_tree->at(root)->paths){
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
        for(auto e : updated_tree->at(root)->paths){
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
bool Router::tree2treeMazeRouting(Net *net, Subtree *source, Subtree *sink){
    /* Declaring */
    bool success = true;
    Vertex *current;
    auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
    std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
    /* Initialize the soruce and sink verteices */
    // Let the second point be the sink
    this->grid->setSinks(std::vector<Coordinate3D>(sink->pinlist.begin(), sink->pinlist.end()));
    for(auto &p : sink->paths){
        for(auto &s : p->segments){
            this->grid->setSinks(*s);
        }
    }
    // Set all vertex's distance to infinity
    this->grid->setDistanceInfinity();
    // Set the source's distance to zero
    this->grid->setDistanceZero(std::vector<Coordinate3D>(source->pinlist.begin(), source->pinlist.end()));
    for(auto p : source->pinlist){
        pq.push(this->grid->graph.at(p.x).at(p.y).at(p.z));
    }
    for(auto &p : source->paths){
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
     * Dijkstra's algorithm for finding the shortest path in tree to tree.
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
    if(!this->grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
        std::cout << "Net#" << net->id << " " << source->showPins() << "- " << sink->showPins() << " need reroute\n";
        success = false;
    }
    // Tree2tree routing success
    else{
        Path *tmp_path = new Path();
        tmp_path->start_pin = current->coordinate;

        source->paths.push_back(tmp_path);

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
                    if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
                    tmp_seg = new Segment(); 
                }
                tmp_seg->z = current->coordinate.z;
                tmp_seg->x = current->coordinate.x;
                tmp_seg->y = current->coordinate.y;
            }
            current->obstacle = net->id;
            if(tmp_path != nullptr) current->cur_paths.push_back(tmp_path);
            current = current->prevertex;
        }

        current->obstacle = net->id;
        if(tmp_path != nullptr) current->cur_paths.push_back(tmp_path);

        if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
            if(tmp_seg->z == 0){
                tmp_seg->neighbor = current->coordinate.x;
            }
            else{
                tmp_seg->neighbor = current->coordinate.y;
            }
            if(tmp_seg != nullptr) tmp_path->segments.push_back(tmp_seg);
        }

        tmp_path->end_pin = current->coordinate;
        /* Split Path */
        std::unordered_set<Path*> path_count;
        path_count.insert(grid->graph.at(tmp_path->start_pin.x).at(tmp_path->start_pin.y).at(tmp_path->start_pin.z)->cur_paths.begin()
                , grid->graph.at(tmp_path->start_pin.x).at(tmp_path->start_pin.y).at(tmp_path->start_pin.z)->cur_paths.end());
        path_count.insert(grid->graph.at(tmp_path->start_pin.x).at(tmp_path->start_pin.y).at((tmp_path->start_pin.z + 1) % 2)->cur_paths.begin()
                , grid->graph.at(tmp_path->start_pin.x).at(tmp_path->start_pin.y).at((tmp_path->start_pin.z + 1) % 2)->cur_paths.end());

        if(path_count.size() == 2){
            if(!source->pinlist.count(tmp_path->start_pin) && !source->pinlist.count(Coordinate3D{tmp_path->start_pin.x, tmp_path->start_pin.y, (tmp_path->start_pin.z + 1) % 2})){
                if(!sink->pinlist.count(tmp_path->start_pin) && !sink->pinlist.count(Coordinate3D{tmp_path->start_pin.x, tmp_path->start_pin.y, (tmp_path->start_pin.z + 1) % 2})){
                    std::unordered_set<Path*> splited; splited.insert(tmp_path);
                    bool split = false;
                    for(auto &split_candidate_path : this->grid->graph.at(tmp_path->start_pin.x).at(tmp_path->start_pin.y).at(tmp_path->start_pin.z)->cur_paths){
                        if(splited.count(split_candidate_path)) continue;
                        splited.insert(split_candidate_path);
                        // Check source->paths contain the tmp_path->start_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->start_pin, split_candidate_path, source->paths, net->id);
                        // Check sink->paths contain the tmp_path->start_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->start_pin, split_candidate_path, sink->paths, net->id);
                        if(split) break;
                    }
                    for(auto &split_candidate_path : this->grid->graph.at(tmp_path->start_pin.x).at(tmp_path->start_pin.y).at((tmp_path->start_pin.z + 1) % 2)->cur_paths){
                        if(splited.count(split_candidate_path)) continue;
                        splited.insert(split_candidate_path);
                        // Check source->paths contain the tmp_path->start_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->start_pin, split_candidate_path, source->paths, net->id);
                        // Check sink->paths contain the tmp_path->start_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->start_pin, split_candidate_path, sink->paths, net->id);
                        if(split) break;
                    }
                }
            }
            
        }
        path_count.clear();
        path_count.insert(grid->graph.at(tmp_path->end_pin.x).at(tmp_path->end_pin.y).at(tmp_path->end_pin.z)->cur_paths.begin()
                , grid->graph.at(tmp_path->end_pin.x).at(tmp_path->end_pin.y).at(tmp_path->end_pin.z)->cur_paths.end());
        path_count.insert(grid->graph.at(tmp_path->end_pin.x).at(tmp_path->end_pin.y).at((tmp_path->end_pin.z + 1) % 2)->cur_paths.begin()
                , grid->graph.at(tmp_path->end_pin.x).at(tmp_path->end_pin.y).at((tmp_path->end_pin.z + 1) % 2)->cur_paths.end());
        if(path_count.size() == 2){
            if(!source->pinlist.count(tmp_path->end_pin) && !source->pinlist.count(Coordinate3D{tmp_path->end_pin.x, tmp_path->end_pin.y, (tmp_path->end_pin.z + 1) % 2})){
                if(!sink->pinlist.count(tmp_path->end_pin) && !sink->pinlist.count(Coordinate3D{tmp_path->end_pin.x, tmp_path->end_pin.y, (tmp_path->end_pin.z + 1) % 2})){
                    std::unordered_set<Path*> splited; splited.insert(tmp_path);
                    bool split = false;
                    for(auto &split_candidate_path :  this->grid->graph.at(tmp_path->end_pin.x).at(tmp_path->end_pin.y).at(tmp_path->end_pin.z)->cur_paths){
                        if(splited.count(split_candidate_path)) continue;
                        // Then check source->paths contain the tmp_path->end_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->end_pin, split_candidate_path, source->paths, net->id);
                        // Then check sink->paths contain the tmp_path->end_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->end_pin, split_candidate_path, sink->paths, net->id);
                        if(split) break;
                    }
                    for(auto &split_candidate_path :  this->grid->graph.at(tmp_path->end_pin.x).at(tmp_path->end_pin.y).at((tmp_path->end_pin.z + 1) % 2)->cur_paths){
                        if(splited.count(split_candidate_path)) continue;
                        // Then check source->paths contain the tmp_path->end_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->end_pin, split_candidate_path, source->paths, net->id);
                        // Then check sink->paths contain the tmp_path->end_pin
                        if(!split) split = splitPaths(this->grid, tmp_path->end_pin, split_candidate_path, sink->paths, net->id);
                        if(split) break;
                    }
                }
            }
        }
    }
    /* Post job cleanup */
    // Let the second point reset to not the sink
    this->grid->resetSinks(std::vector<Coordinate3D>(sink->pinlist.begin(), sink->pinlist.end()));
    for(auto &p : sink->paths){
        for(auto &s : p->segments){
            this->grid->resetSinks(*s);
        }
    }
    return success;
}
/* 
 * Tree2tree maze routing, return the path from source to sink
 * If not success will throw runtime error.
 */
Path Router::tree2treeMazeRouting(Grid *tmp_grid, Net *net, Subtree *source, Subtree *sink){
    /* Declaring */
    Vertex *current;
    auto comp = [](const Vertex *lhs, const Vertex *rhs) {return lhs->distance > rhs->distance;};
    std::priority_queue<Vertex*, std::vector<Vertex*>, decltype(comp)> pq(comp);
    /* Initialize the soruce and sink verteices */
    // Let the second point be the sink
    tmp_grid->setSinks(std::vector<Coordinate3D>(sink->pinlist.begin(), sink->pinlist.end()));
    for(auto &p : sink->paths){
        for(auto &s : p->segments){
            tmp_grid->setSinks(*s);
        }
    }
    // Set all vertex's distance to infinity
    tmp_grid->setDistanceInfinity();
    // Set the source's distance to zero
    tmp_grid->setDistanceZero(std::vector<Coordinate3D>(source->pinlist.begin(), source->pinlist.end()));
    for(auto p : source->pinlist){
        pq.push(tmp_grid->graph.at(p.x)
            .at(p.y).at(p.z));
    }
    for(auto &p : source->paths){
        for(auto &s : p->segments){
            if(s->z == 0){
                for(int i = std::min(s->x, s->neighbor); i <= std::max(s->x, s->neighbor); i++){
                    if(tmp_grid->graph.at(i).at(s->y).at(s->z)->distance != 0){
                        pq.push(tmp_grid->graph.at(i).at(s->y).at(s->z));
                    }
                }
            }
            else{
                for(int i = std::min(s->y, s->neighbor); i <= std::max(s->y, s->neighbor); i++){
                    if(tmp_grid->graph.at(s->x).at(i).at(s->z)->distance != 0){
                        pq.push(tmp_grid->graph.at(s->x).at(i).at(s->z));
                    }
                }
            }
            tmp_grid->setDistanceZero(*s);
        }
    }
    // Set all vertex's prevertex to nullptr
    tmp_grid->setPrevertexNull();
    /* 
     * Dijkstra's algorithm for finding the shortest path in tree to tree.
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
            if(tmp_grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->isObstacle()
            && !tmp_grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->is_sink) continue;
            if(current->distance + mazeRouteCost(this, current->coordinate, Coordinate3D{current->coordinate.x + move_orientation.at(cur_z).at(i).x, current->coordinate.y + move_orientation.at(cur_z).at(i).y, cur_z + move_orientation.at(cur_z).at(i).z})
                    < tmp_grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->distance){
                tmp_grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->prevertex = current;
                tmp_grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z)->distance 
                    = current->distance + mazeRouteCost(this, current->coordinate, Coordinate3D{current->coordinate.x + move_orientation.at(cur_z).at(i).x, current->coordinate.y + move_orientation.at(cur_z).at(i).y, cur_z + move_orientation.at(cur_z).at(i).z}); 
                pq.push(tmp_grid->graph.at(current->coordinate.x + move_orientation.at(cur_z).at(i).x).at(current->coordinate.y + move_orientation.at(cur_z).at(i).y).at(cur_z + move_orientation.at(cur_z).at(i).z));
            }
        }
    }
    /* Backtracking */
    // Failed tree2tree routing
    if(!tmp_grid->graph.at(current->coordinate.x).at(current->coordinate.y).at(current->coordinate.z)->is_sink){
        throw std::runtime_error("Error: reroute Net#" + std::to_string(net->id) + " " + source->showPins() + "- " + sink->showPins() + "\n");
    }
    // Tree2tree routing success
    Path tmp_path;
    tmp_path.start_pin = current->coordinate;

    Segment *tmp_seg = nullptr;
    while(current->prevertex != nullptr){
        if(tmp_seg == nullptr){
            tmp_seg = new Segment();
            tmp_seg->z = current->coordinate.z;
            tmp_seg->x = current->coordinate.x;
            tmp_seg->y = current->coordinate.y;
        }
        if(tmp_seg->z != current->coordinate.z){
            // tmp_grid->graph.at(current->coordinate.x).at(current->coordinate.y).at((current->coordinate.z + 1) % 2)->obstacle = net->id;
            if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
                if(tmp_seg->z == 0){
                    tmp_seg->neighbor = current->coordinate.x;
                }
                else{
                    tmp_seg->neighbor = current->coordinate.y;
                }
                if(tmp_seg != nullptr) tmp_path.segments.push_back(tmp_seg);
                tmp_seg = new Segment(); 
            }
            tmp_seg->z = current->coordinate.z;
            tmp_seg->x = current->coordinate.x;
            tmp_seg->y = current->coordinate.y;
        }
        // current->obstacle = net->id;
        current = current->prevertex;
    }
    // current->obstacle = net->id;

    if(tmp_seg->x != current->coordinate.x || tmp_seg->y != current->coordinate.y){
        if(tmp_seg->z == 0){
            tmp_seg->neighbor = current->coordinate.x;
        }
        else{
            tmp_seg->neighbor = current->coordinate.y;
        }
        if(tmp_seg != nullptr) tmp_path.segments.push_back(tmp_seg);
    }

    tmp_path.end_pin = current->coordinate;
    /* Post job cleanup */
    // Let the second point reset to not the sink
    tmp_grid->resetSinks(std::vector<Coordinate3D>(sink->pinlist.begin(), sink->pinlist.end()));
    for(auto &p : sink->paths){
        for(auto &s : p->segments){
            tmp_grid->resetSinks(*s);
        }
    }
    return tmp_path;
}