#include <algorithm>
#include <iostream>
#include <tuple>
#include <deque>
#include "router.hpp"
// debug
int gcnt = 0;
// debug
void Router::routing(Net &n, int source_index, int sink_index){
    std::deque<std::tuple<Net*, int, int>> rip_up_pair;
    if(n.tree->find(source_index) == n.tree->find(sink_index)){
        for(unsigned i = 0; i < n.tree->parents.size(); i++){
            if(n.tree->find(i) != n.tree->find(source_index)){
                sink_index = i;
                break;
            }
        }
    }
    if(n.tree->find(source_index) == n.tree->find(sink_index)) return;
    if(!tree2treeMazeRouting(&n, n.tree->at(source_index), n.tree->at(sink_index))){
        num_of_reroute++;
        Path tmp_path = tree2treeMazeRouting(pin_and_obstacle_grid, &n, n.tree->at(source_index), n.tree->at(sink_index));
        addHistoryCost(&tmp_path);
        // debug
        std::cout << "Net#" << n.id << "\n";
        for(auto i : tmp_path.segments){
            std::cout << i->startPoint().toString() << "-" << i->endPoint().toString() << "\n";
        }
        std::cout << "\n";
        // debug
        Path *rip_up_candidate = nullptr;
        do{
            rip_up_candidate = nullptr;
            for(auto s : tmp_path.segments){
                if(s->z == 0){
                    for(int i = s->getX(); i <= s->getNeighbor(); i++){
                        if(grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths.size() > 0){
                            rip_up_candidate = grid->graph.at(i).at(s->getY()).at(s->z)->cur_paths.at(0);
                            if(rip_up_candidate != nullptr){
                                if(rip_up_candidate != nullptr) addHistoryCost(rip_up_candidate);
                                auto &current_net = layout->netlist.at(grid->graph.at(i).at(s->getY()).at(s->z)->obstacle);
                                const auto &[souce_index, sink_index] = ripUpPaths(grid, rip_up_candidate, current_net.tree);
                                rip_up_pair.push_back(std::make_tuple(&current_net, souce_index, sink_index));
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
                                if(rip_up_candidate != nullptr) addHistoryCost(rip_up_candidate);
                                auto &current_net = layout->netlist.at(grid->graph.at(s->getX()).at(i).at(s->z)->obstacle);
                                const auto &[souce_index, sink_index] = ripUpPaths(grid, rip_up_candidate, current_net.tree);
                                rip_up_pair.push_back(std::make_tuple(&current_net, souce_index, sink_index));
                            }
                            break;
                        }
                    }
                }
                if(rip_up_candidate != nullptr) break;
            }
        }while(rip_up_candidate != nullptr);

        rip_up_pair.push_back(std::make_tuple(&n, source_index, sink_index));
    }
    else if(!n.tree->mergeTree(source_index, sink_index)) {
        throw std::runtime_error("Error: merge tree error");
    }
    for(auto rup : rip_up_pair){
        const auto &[current_net, souce_index, sink_index] = rup;
        if(current_net->id == 102){

        }
        this->routing(*current_net, souce_index, sink_index);
    }
}
void Router::main(){
    this->twoPinNetDecomposition();
    for(auto &n : layout->netlist){
        n.initTrees();
        for(auto &tpn : n.two_pins_net){
            this->routing(n, tpn.first, tpn.second);
            // reset history cost
            for(unsigned i = 0; i < grid->history.size(); i++){
                for(unsigned j = 0; j < grid->history.at(i).size(); j++){
                    for(unsigned k = 0; k < grid->history.at(i).at(j).size(); k++){
                        grid->history.at(i).at(j).at(k) = 0.0;
                    }
                }
            }
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