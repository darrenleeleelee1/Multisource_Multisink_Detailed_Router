#pragma once
#include <vector>
#include <set>
class Coordinate3D{
public:
	int x, y, z;
	Coordinate3D(){}
	Coordinate3D(int _x, int  _y, int  _z) : x(_x), y(_y), z(_z) {}
	~Coordinate3D(){}
	bool operator==(const Coordinate3D &other) const {
    	return this->x == other.x && this->y == other.y && this->z == other.z;
    }
};
class Coordinate2D{
public:
	int x, y;
	Coordinate2D(){}
	Coordinate2D(int _x, int  _y) : x(_x), y(_y) {}
	~Coordinate2D(){}
	bool operator==(const Coordinate2D &other) const {
    	return this->x == other.x && this->y == other.y;
    }
};
class Obstacle{
public:
	Coordinate3D start_point, end_point;
	Obstacle(){}
	Obstacle(int _spx, int  _spy, int  _spz, int _epx, int  _epy, int  _epz)
		: start_point(_spx, _spy, _spz), end_point(_epx, _epy, _epz) {}
	~Obstacle(){}
	bool operator==(const Obstacle &other) const {
    	return this->start_point == other.start_point && this->end_point == other.end_point;
    }
};
class Segment_Draw{
public:
	// If end_point == 0, means this segment is horizontal, sp.y == ep.y && sp.z == ep.z == 0
	// else if end_point == 1, means this segment is vertical, sp.x == ep.x && sp.z == ep.z == 1
	Coordinate3D start_point, end_point;
	Segment_Draw(){}
	Segment_Draw(int _spx, int  _spy, int  _spz, int _epx, int  _epy, int  _epz)
		: start_point(std::min(_spx, _epx), std::min(_spy, _epy), _spz)
			, end_point(std::max(_spx, _epx), std::max(_spy, _epy), _epz) {}
	~Segment_Draw(){}
	bool operator==(const Segment_Draw &other) const {
    	return this->start_point == other.start_point && this->end_point == other.end_point;
    }
};
class Segment{
public:
	int attribute; // 0 for horizon, 1 for verticle
	int x, y, neighbor;
	Segment(){}
	Segment(int _attr, int _x, int _y, int nei) : attribute(_attr), x(_x), y(_y), neighbor(nei) {}
};
class Path{
public:
	std::vector<Segment> segments;
	std::pair<Coordinate3D, Coordinate3D> pin_pair;
	Path() {
		this->segments.resize(0);
	}
	~Path() {}
};
class Subtree{
public:
	std::vector<Path> paths;
	std::set<int> pins;
	Subtree() {
		this->pins.clear();
		this->paths.resize(0);
	}
	~Subtree() {}
};
class Net{
public:
	// For Input
	int id;
	std::vector<Coordinate3D> pins;
	// For Ouput
	std::vector<Coordinate2D> vialist;
	std::vector<Segment_Draw> horizontal_segments;
	std::vector<Segment_Draw> vertical_segments;
	// For Router
	std::vector<std::pair<int, int>> two_pins_net;
	std::vector<Subtree> subtrees;
	Net(){
		id = -1;
		this->pins.resize(0);
		this->vialist.resize(0);
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
		this->subtrees.resize(0);
	}
	Net(int _id) : id(_id){
		this->pins.resize(0);
		this->vialist.resize(0);
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
		this->subtrees.resize(0);
	}
	~Net(){}

	int getWirelength(){
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

	int getWirelength(){
		int sum = 0;
		for(unsigned i = 0; i < this->netlist.size(); i++){
			sum += this->netlist.at(i).getWirelength();
		}
		return sum;
	}
};