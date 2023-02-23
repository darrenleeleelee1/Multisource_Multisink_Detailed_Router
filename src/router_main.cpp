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
    for(auto &n : layout->netlist){
        n.initTrees();
        for(auto &tpn : n.two_pins_net){
            int reroute_state = 0; // 1 means source stuck, 2 means sink stuck
            if(n.tree->at(tpn.first)->pinlist.size() > n.tree->at(tpn.second)->pinlist.size()) std::swap(tpn.first, tpn.second);
            if(!tree2tree_maze_routing(&n, n.tree->at(tpn.first), n.tree->at(tpn.second), reroute_state)){
                // TODO reroute
                if(reroute_state == 1){
                    auto &reroute_vertex = n.pins.at(tpn.first);
                    for(unsigned i = 0; i < move_orientation.at(reroute_vertex.z).size(); i++){
                        auto &current_vertex = grid->graph.at(reroute_vertex.x + move_orientation.at(reroute_vertex.z).at(i).x)
                                .at(reroute_vertex.y + move_orientation.at(reroute_vertex.z).at(i).y).at(reroute_vertex.z + move_orientation.at(reroute_vertex.z).at(i).z);
                        if(static_cast<unsigned>(current_vertex->obstacle) == layout->netlist.size()) continue; // Is obstacles
                        auto &current_net = layout->netlist.at(current_vertex->obstacle);
                        if(!current_net.checkIsPin(current_vertex->coordinate)){
                            std::cout << "1: " << current_vertex->cur_paths.size() << current_vertex->coordinate.toString() << "\n";
                        }
                    }
                }
                else if(reroute_state == 2){
                    auto &reroute_vertex = n.pins.at(tpn.second);
                    for(unsigned i = 0; i < move_orientation.at(reroute_vertex.z).size(); i++){
                        auto &current_vertex = grid->graph.at(reroute_vertex.x + move_orientation.at(reroute_vertex.z).at(i).x)
                                .at(reroute_vertex.y + move_orientation.at(reroute_vertex.z).at(i).y).at(reroute_vertex.z + move_orientation.at(reroute_vertex.z).at(i).z);
                        if(static_cast<unsigned>(current_vertex->obstacle) == layout->netlist.size()) continue; // Is obstacles
                        auto &current_net = layout->netlist.at(current_vertex->obstacle);
                        if(!current_net.checkIsPin(current_vertex->coordinate)){
                            erasePaths(grid, current_vertex->cur_paths.at(0), current_net.tree);
                            std::cout << "2: " << current_vertex->cur_paths.size() << current_vertex->coordinate.toString() << "\n";
                        }
                    }
                }
            }
            else {
                if (!n.tree->mergeTree(tpn.first, tpn.second)) {
                    throw std::runtime_error("Error: merge tree failed");
                }
            }
        }
    }
}