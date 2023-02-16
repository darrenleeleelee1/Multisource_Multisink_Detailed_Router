#include <algorithm>
#include <iostream>
#include "router.hpp"
void Router::main(){
    this->twoPinNetDecomposition();

    // sort Nets, try to add some heuristic
    // first pick the pins number, from the large to the small
    // then pick the bounding box, from the large to the small
    // std::sort(this->layout->netlist.begin(), this->layout->netlist.end(), [](const Net& a, const Net& b){
    //     if(a.pins.size() == b.pins.size()){
    //         int min_a_x = a.pins.at(0).x, max_a_x = a.pins.at(0).x;
    //         int min_b_x = b.pins.at(0).x, max_b_x = b.pins.at(0).x;
    //         for (const auto& pin : a.pins) {
    //             min_a_x = std::min(min_a_x, pin.x);
    //             max_a_x = std::max(max_a_x, pin.x);
    //         }
    //         for (const auto& pin : b.pins) {
    //             min_b_x = std::min(min_b_x, pin.x);
    //             max_b_x = std::max(max_b_x, pin.x);
    //         }

    //         int min_a_y = a.pins.at(0).x, max_a_y = a.pins.at(0).x;
    //         int min_b_y = b.pins.at(0).x, max_b_y = b.pins.at(0).x;
    //         for (const auto& pin : a.pins) {
    //             min_a_y = std::min(min_a_y, pin.x);
    //             max_a_y = std::max(max_a_y, pin.x);
    //         }
    //         for (const auto& pin : b.pins) {
    //             min_b_y = std::min(min_b_y, pin.x);
    //             max_b_y = std::max(max_b_y, pin.x);
    //         }

    //         return (max_a_x - min_a_x) * (max_a_y - min_a_y) < (max_b_x - min_b_x) * (max_b_y - min_b_y);
    //     }
    //     else return a.pins.size() > b.pins.size();
    // });

    for(auto &n : this->layout->netlist){
        for(auto &tpn : n.two_pins_net){
            // Because Router constructor mark all the net pins to obstacle
            // Mark current net pins not a obstacle
            // this->grid->resetObstacles(n.pins);
            int reroute_state = 0; // 1 means source stuck, 2 means sink stuck
            if(!this->tree2tree_maze_routing(&n, n.pins.at(tpn.first), n.pins.at(tpn.second), reroute_state)){
                // bool reroute_candidate = false;
                // Coordinate3D &reroute_node = (reroute_state == 1 ? n.pins.at(tpn.first) : n.pins.at(tpn.second));
                // for(int i = 0; i < 4; i++){
                //     if(outOfBound(Coordinate3D{reroute_node.x + this->x_orientation.at(i), reroute_node.y+ this->y_orientation.at(i), i % 2})) continue;
                //     // TODO: Check four direction that which is segment(only if not a pins, if could be reroute)
                //     auto &next_vertex = this->grid->graph.at(reroute_node.x + this->x_orientation.at(i)).at(reroute_node.y+ this->y_orientation.at(i)).at(i % 2);
                //     std::cout << "reroute segment size: "<< next_vertex->cur_segments.size() << "\n";
                //     if(next_vertex->isObstacle()){
                //         bool not_pin_location = true;
                //         for(auto &p : this->layout->netlist.at(next_vertex->obstacle).pins){
                //             if(p == next_vertex->coordinate) not_pin_location = false;
                //         }
                //         if(!not_pin_location) continue;
                //         for(auto &p : this->layout->netlist.at(next_vertex->obstacle).paths){
                //             for (auto s = p->segments.begin(); s != p->segments.end(); ++s) {
                //                 for (auto cs = next_vertex->cur_segments.begin(); cs != next_vertex->cur_segments.end(); ++cs) {
                //                     if (*s == *cs) {
                //                         std::cout << "Find segment\n";
                //                         std::cout << "Rip-up Net#" << n.id << " " << (*cs)->toString() << "\n";
                                        
                //                         this->grid->resetObstacles(**cs);
                                        
                //                         if(!this->tree2tree_maze_routing(&n, n.pins.at(tpn.first), n.pins.at(tpn.second), reroute_state)){
                //                             std::cout << "Reroute success\n";
                //                         }
                //                         if(!this->pin2pin_maze_routing(&n, Coordinate3D{(*cs)->}))
                //                         // If the segment is found, erase s from the vector
                //                         p->segments.erase(s);
                //                         delete *s;
                //                         reroute_candidate = true;
                //                         break; // Exit the loop since we found the segment
                //                     }
                //                 }
                //                 if(reroute_candidate) break;
                //             }
                //             if(reroute_candidate) break;
                //         }
                //     }
                //     if(reroute_candidate) break;
                // }
            }
            // When this net route success, turn the net pins into obstacles
            // this->grid->setObstacles(n.id, n.pins);
        }
    }

}