#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "layout.hpp"

std::stringstream ss;

namespace io{

void tokenLine(std::vector<std::string> &tokens, std::string line){
    ss.clear(); ss.str(line);
    tokens.clear(); tokens.resize(0);
    std::string intermediate;
    while(true){
        ss >> intermediate;
        if(ss.fail()) break;
        tokens.push_back(intermediate);
    }
}

void readLayout(Layout *layout, char const *file_path)
{
    std::ifstream in_file(file_path);
    std::string line;
    std::vector<std::string> tokens;
    while(getline(in_file, line)){
        tokenLine(tokens, line);
        if(tokens.size() == static_cast<unsigned int>(2)){
            if(tokens[0] == "Width"){
                layout->width = stoi(tokens[1]);
            }
            else if(tokens[0] == "Height"){
                layout->height = stoi(tokens[1]);
            }
            else if(tokens[0] == "Layer"){
                layout->num_of_layers = stoi(tokens[1]);
            }
            else if(tokens[0] == "Obstacle_num"){
                layout->num_of_obstacles = stoi(tokens[1]);
                for(int i = 0; i < layout->num_of_obstacles; i++){
                    getline(in_file, line), tokenLine(tokens, line);
                    layout->addObstacle(stoi(tokens[0]), stoi(tokens[1]), stoi(tokens[2])
                                        , stoi(tokens[3]), stoi(tokens[4]), stoi(tokens[5]));
                }
            }
            else if(tokens[0] == "Net_num"){
                layout->num_of_nets = stoi(tokens[1]);
                Net tmp_net;
                getline(in_file, line), tokenLine(tokens, line);
                tmp_net.id = stoi(tokens[1]);
                getline(in_file, line), tokenLine(tokens, line);
                tmp_net.num_of_pins = stoi(tokens[1]);
                for(int i = 0; i < tmp_net.num_of_pins; i++){
                    getline(in_file, line), tokenLine(tokens, line);
                    tmp_net.addPin(stoi(tokens[0]), stoi(tokens[1]), stoi(tokens[2]));
                }
            }
            else if(tokens[0] == "Via_cost"){
                layout->via_cost = stoi(tokens[1]);
            }
            else if(tokens[0] == "Horizontal_segment_cost"){
                layout->horizontal_segment_cost = stoi(tokens[1]);
            }
            else if(tokens[0] == "Vertical_segment_cost"){
                layout->vertical_segment_cost = stoi(tokens[1]);
            }
        }
    }
}
void writeLayout(Layout *layout, char const *file_path)
{
    std::ofstream out_file(file_path, std::ofstream::trunc);
    out_file << "Width " << layout->width << "\n";
    out_file << "Height " << layout->height << "\n";
    out_file << "Layer " << layout->num_of_layers << "\n";
    out_file << "Obstacle_num " << layout->num_of_obstacles << "\n";
    for(int i = 0; i < layout->num_of_obstacles; i++){
        out_file << layout->obstacles.at(i).start_point.x << " " << layout->obstacles.at(i).start_point.y << " " << layout->obstacles.at(i).start_point.z; 
        out_file << layout->obstacles.at(i).end_point.x << " " << layout->obstacles.at(i).end_point.y << " " << layout->obstacles.at(i).end_point.z;
        out_file << "\n";
    }
    out_file << "Net_num " << layout->num_of_nets << "\n";
    for(int i = 0; i < layout->num_of_nets; i++){
        out_file << "Net_id " << layout->netlist.at(i).id << "\n";
        out_file << "pin_num " << layout->netlist.at(i).num_of_pins << "\n";
        for(int j = 0; j < layout->netlist.at(i).num_of_pins; j++){
            out_file << layout->netlist.at(i).pins.at(j).x << " " << layout->netlist.at(i).pins.at(j).y << layout->netlist.at(i).pins.at(j).z << "\n";
        }
    }
}
} // namespace io

