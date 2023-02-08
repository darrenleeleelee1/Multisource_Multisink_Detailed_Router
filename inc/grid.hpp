#pragma once
#include <vector>
#include <cstdint>
#include "layout.hpp"
class Vertex{
public:
    Coordinate3D coordinate;
    Vertex *prevertex = nullptr;
    int distance = INT32_MAX;
    bool is_obstacle = false;
    bool is_sink = false;
    Vertex(){}
    Vertex(int x, int y, int z, bool o, bool s) 
        : coordinate(x, y, z), is_obstacle(o), is_sink(s) {}
    ~Vertex() {}
};
class Grid{
public:
    std::vector<std::vector<std::vector<Vertex*>>> graph;
    Grid() {}
    Grid(int width, int height){
        int z_depth = 2;
        
        graph.resize(static_cast<unsigned>(width + 1));
        
        for(int i = 0; i <= width; i++){
            graph.at(i).resize(height + 1);
        }
        
        for(int i = 0; i <= width; i++){
            for(int j = 0; j <= height; j++){
                graph.at(i).at(j).resize(z_depth);
                for(int k = 0; k < z_depth; k++){
                    graph.at(i).at(j).at(k) = new Vertex{i, j, k, false, false};
                }
            }
        }
    }
    ~Grid() {
        for (unsigned i = 0; i < graph.size(); i++) {
            for (unsigned j = 0; j < graph.at(i).size(); j++) {
                for (unsigned k = 0; k < graph.at(i).at(j).size(); k++) {
                    delete graph.at(i).at(j).at(k);
                }
            }
        }

    }
    void setPrevertexNull(){
        for (unsigned i = 0; i < graph.size(); i++) {
            for (unsigned j = 0; j < graph[i].size(); j++) {
                for (unsigned k = 0; k < graph[i][j].size(); k++) {
                    this->graph.at(i).at(j).at(k)->prevertex = nullptr;
                }
            }
        }
    }
    void setDistanceInfinity(){
        for (unsigned i = 0; i < graph.size(); i++) {
            for (unsigned j = 0; j < graph[i].size(); j++) {
                for (unsigned k = 0; k < graph[i][j].size(); k++) {
                    this->graph.at(i).at(j).at(k)->distance = INT32_MAX;
                }
            }
        }
    }
    void setDistanceZero(Coordinate3D coor){
        this->graph.at(coor.x).at(coor.y).at(coor.z)->distance = 0;
    }
    void setDistanceZero(Segment seg){
        if(seg.attribute == 0){
            for(int i = std::min(seg.x, seg.neighbor); i <= std::max(seg.x, seg.neighbor); i++){
                this->graph.at(i).at(seg.y).at(seg.attribute)->distance = 0;
            }
        }
        else{
            for(int i = std::min(seg.y, seg.neighbor); i <= std::max(seg.y, seg.neighbor); i++){
                this->graph.at(seg.x).at(i).at(seg.attribute)->distance = 0;
            }
        }
    }
    void setObstacles(Coordinate3D start_point, Coordinate3D end_point){
        // pins
        if(start_point == end_point){
            this->graph.at(start_point.x).at(start_point.y).at(start_point.z)->is_obstacle = true;
        }
        else{
            // Horizontal
            if(start_point.y == end_point.y){
                for(int i = std::min(start_point.x, end_point.x); i <= std::max(start_point.x, end_point.x); i++){
                    this->graph.at(i).at(start_point.y).at(start_point.z)->is_obstacle = true;
                }
            }
            // Verticle
            else if(start_point.x == end_point.x){
                for(int i = std::min(start_point.y, end_point.y); i <= std::max(start_point.y, end_point.y); i++){
                    this->graph.at(start_point.x).at(i).at(start_point.z)->is_obstacle = true;
                }
            }
        }
    }
    void setObstacles(std::vector<Coordinate3D> pins){
        for(auto v : pins){
            this->graph.at(v.x).at(v.y).at(v.z)->is_obstacle = true;
        }
    }
    void resetObstacles(std::vector<Coordinate3D> pins){
        for(auto v : pins){
            this->graph.at(v.x).at(v.y).at(v.z)->is_obstacle = false;
        }
    }
    void setSinks(Segment seg){
        if(seg.attribute == 0){
            for(int i = std::min(seg.x, seg.neighbor); i <= std::max(seg.x, seg.neighbor); i++){
                this->graph.at(i).at(seg.y).at(seg.attribute)->is_sink = true;
            }
        }
        else{
            for(int i = std::min(seg.y, seg.neighbor); i <= std::max(seg.y, seg.neighbor); i++){
                this->graph.at(seg.x).at(i).at(seg.attribute)->is_sink = true;
            }
        }
    }
    void setSinks(Coordinate3D coor){
        this->graph.at(coor.x).at(coor.y).at(coor.z)->is_sink = true;
    }
    void setSinks(std::vector<Coordinate3D> pins){
        for(auto v : pins){
            this->graph.at(v.x).at(v.y).at(v.z)->is_sink = true;
        }
    }
    void resetSinks(Segment seg){
        if(seg.attribute == 0){
            for(int i = std::min(seg.x, seg.neighbor); i <= std::max(seg.x, seg.neighbor); i++){
                this->graph.at(i).at(seg.y).at(seg.attribute)->is_sink = false;
            }
        }
        else{
            for(int i = std::min(seg.y, seg.neighbor); i <= std::max(seg.y, seg.neighbor); i++){
                this->graph.at(seg.x).at(i).at(seg.attribute)->is_sink = false;
            }
        }
    }
    void resetSinks(Coordinate3D coor){
        this->graph.at(coor.x).at(coor.y).at(coor.z)->is_sink = false;
    }
    void resetSinks(std::vector<Coordinate3D> pins){
        for(auto v : pins){
            this->graph.at(v.x).at(v.y).at(v.z)->is_sink = false;
        }
    }
};
