#include <algorithm>
#include <iostream>
#include <tuple>
#include <deque>
#include "router.hpp"
void Router::routing(Net &n, int source_index, int sink_index){
    std::deque<std::tuple<Net*, int, int>> rip_up_pair;
    if(!tree2treeMazeRouting(&n, n.tree->at(source_index), n.tree->at(sink_index))){
        Path tmp_path = tree2treeMazeRouting(pin_and_obstacle_grid, &n, n.tree->at(source_index), n.tree->at(sink_index));

        Path *rip_up_candidate = nullptr;
        do{
            rip_up_candidate = nullptr;
            for(auto s : tmp_path.segments){
                if(s->z == 0){
                    for(int i = s->getX(); i <= s->getNeighbor(); i++){
                        if(grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths.size() > 0){
                            rip_up_candidate = grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths.at(0);
                            if(rip_up_candidate != nullptr){
                                auto &current_net = layout->netlist.at(grid->graph.at(i).at(s->getY()).at(s->z)->obstacle);
                                const auto &[souce_index, sink_index] = ripUpPaths(grid, rip_up_candidate, current_net.tree);
                                rip_up_pair.push_front(std::make_tuple(&current_net, souce_index, sink_index));
                            }
                            break;
                        }
                    }
                }
                else if(s->z == 1){
                    for(int i = s->getY(); i <= s->getNeighbor(); i++){
                        if(grid->graph.at(s->getX()).at(i).at(s->z)->cur_paths.size() > 0){
                            rip_up_candidate = grid->graph.at(s->getX()).at(i).at(s->z)->cur_paths.at(0);
                            if(rip_up_candidate != nullptr){
                                auto &current_net = layout->netlist.at(grid->graph.at(s->getX()).at(i).at(s->z)->obstacle);
                                const auto &[souce_index, sink_index] = ripUpPaths(grid, rip_up_candidate, current_net.tree);
                                rip_up_pair.push_front(std::make_tuple(&current_net, souce_index, sink_index));
                            }
                            break;
                        }
                    }
                }
                if(rip_up_candidate != nullptr) break;
            }
            
        }while(rip_up_candidate != nullptr);


        rip_up_pair.push_front(std::make_tuple(&n, source_index, sink_index));
    }
    else if(!n.tree->mergeTree(source_index, sink_index)) {
        throw std::runtime_error("Error: merge tree error");
    }
    for(auto rup : rip_up_pair){
        const auto &[current_net, souce_index, sink_index] = rup;
        if(!tree2treeMazeRouting(current_net, current_net->tree->at(souce_index), current_net->tree->at(sink_index))){
            throw std::runtime_error("Error: need more rip-up");
        }
        else if (!current_net->tree->mergeTree(souce_index, sink_index)) {
            throw std::runtime_error("Error: merge tree error");
        }
    }
}
void Router::main(){
    this->twoPinNetDecomposition();

    
    for(auto &n : layout->netlist){
        n.initTrees();
        for(auto &tpn : n.two_pins_net){
            this->routing(n, tpn.first, tpn.second);
        }
        /* TESTING path is correct set on the grid */
        for(auto e : n.tree->getPath()){
            bool find = false;
            for(auto t : grid->graph.at(e->start_pin.x).at(e->start_pin.y).at(e->start_pin.z)->cur_paths){
                if(t == e) find = true;
            }
            if(!find) {
                std::cout << "Error: Net#" + std::to_string(n.id) <<  " Path#" << e->start_pin.toString() << " - " 
                    << e->end_pin.toString() << " Grid not contain " + Coordinate3D{e->start_pin.x, e->start_pin.y, e->start_pin.z}.toString() + "\n";
                throw std::runtime_error("Read the error message");
            }
            find = false;
            for(auto t : grid->graph.at(e->end_pin.x).at(e->end_pin.y).at(e->end_pin.z)->cur_paths){
                if(t == e) find = true;
            }
            if(!find) {
                std::cout << "Error: Net#" + std::to_string(n.id) <<  " Path#" << e->start_pin.toString() << " - " 
                    << e->end_pin.toString() << " Grid not contain " + Coordinate3D{e->end_pin.x, e->end_pin.y, e->end_pin.z}.toString() + "\n";
                throw std::runtime_error("Read the error message");
            }

            for(auto s : e->segments){
                if(s->z == 0){
                    for(int i = s->getX(); i <= s->getNeighbor(); i++){
                        for(auto t : grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths){
                            if(t == e) find = true;
                        }
                        if(!find){
                            std::cout << ("Error: Net#" + std::to_string(n.id) + " Grid not set " + s->toString() + " well\n");
                            throw std::runtime_error("Read the error message");
                        }
                    }
                }
                else if(s->z == 1){
                    for(int i = s->getY(); i <= s->getNeighbor(); i++){
                        for(auto t : grid->graph.at(s->getX()).at(i).at(s->z)->cur_paths){
                            if(t == e) find = true;
                        }
                        if(!find){
                            std::cout << ("Error: Net#" + std::to_string(n.id) + " Grid not set " + s->toString() + " well\n");
                            throw std::runtime_error("Read the error message");
                        }
                    }
                }
            }
        }
    }
    
}