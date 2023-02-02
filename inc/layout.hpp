#pragma once
#include <vector>
class Coordinate3D{
public:
	int x, y, z;
	Coordinate3D(){}
	Coordinate3D(int _x, int  _y, int  _z) : x(_x), y(_y), z(_z) {}
	~Coordinate3D(){}
};
class Coordinate2D{
public:
	int x, y;
	Coordinate2D(){}
	Coordinate2D(int _x, int  _y) : x(_x), y(_y) {}
	~Coordinate2D(){}
};
class Obstacle{
public:
	Coordinate3D start_point, end_point;
	Obstacle(){}
	Obstacle(int _spx, int  _spy, int  _spz, int _epx, int  _epy, int  _epz)
		: start_point(_spx, _spy, _spz), end_point(_epx, _epy, _epz) {}
	~Obstacle(){}
};
class Segment{
public:
	// If end_point == 0, means this segment is horizontal, sp.y == ep.y && sp.z == ep.z == 0
	// else if end_point == 1, means this segment is vertical, sp.x == ep.x && sp.z == ep.z == 1
	Coordinate3D start_point, end_point;
	Segment(){}
	Segment(int _spx, int  _spy, int  _spz, int _epx, int  _epy, int  _epz)
		: start_point(_spx, _spy, _spz), end_point(_epx, _epy, _epz) {}
	~Segment(){}
};
class Net{
public:
	// For Input
	int id;
	std::vector<Coordinate3D> pins;
	// For Ouput
	std::vector<Coordinate2D> vialist;
	std::vector<Segment> horizontal_segments;
	std::vector<Segment> vertical_segments;
	// For Router
	std::vector<std::pair<int, int>> two_pins;
	Net(){
		id = -1;
		this->pins.resize(0);
		this->vialist.resize(0);
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
	}
	Net(int _id) : id(_id){
		this->pins.resize(0);
		this->vialist.resize(0);
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
	}
	~Net(){}

	int64_t getWirelength(){
		// TODO compute the wirelength
		return 0;
	}

	void rmst_kruskal(int via_cost, int horizontal_segments_cost, int vertical_segment_cost);
};
class Layout{
public:
	// For Input
	int width, height;
	int num_of_layers = 2;
	int via_cost = 10;
	int horizontal_segment_cost = 1, vertical_segment_cost = 1;
	std::vector<Obstacle> obstacles;
	std::vector<Net> netlist;
	
	Layout(){}
	Layout(int w, int h, int nol, int vc, int hsc, int vsc)
		: width(w), height(h), num_of_layers(nol), via_cost(vc)
			, horizontal_segment_cost(hsc), vertical_segment_cost(vsc) {}
	~Layout(){}

	int64_t getWirelength(){
		int64_t sum = 0;
		for(unsigned i = 0; i < this->netlist.size(); i++){
			sum += this->netlist.at(i).getWirelength();
		}
		return sum;
	}
};