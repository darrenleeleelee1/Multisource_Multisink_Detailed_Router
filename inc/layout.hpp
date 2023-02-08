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
	bool operator<(const Coordinate2D &other) const {
    	if (this->x == other.x)
        	return this->y < other.y;
        return this->x < other.x;
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
	Segment(int _attr, int _x, int _y, int nei) : attribute(_attr){
		if(_attr == 0){
			this->x = std::min(_x, nei);
			this->y = _y;
			this->neighbor = std::max(_x, nei);
		}
		else{
			this->x = _x;
			this->y = std::min(_y, nei);
			this->neighbor = std::max(_y, nei);
		}
	}
};
class Path{
public:
	std::vector<Segment*> segments;
	Coordinate3D start_pin, end_pin;
	Path() {
		this->segments.resize(0);
	}
	~Path() {
		for (auto s : segments) {
			delete s;
		}
	}
};
class Net{
public:
	// For Input
	int id;
	std::vector<Coordinate3D> pins;
	// For Ouput
	std::set<Coordinate2D> vialist;
	// std::vector<Segment> segments;
	// Segment for drawing
	std::vector<Segment_Draw> horizontal_segments;
	std::vector<Segment_Draw> vertical_segments;
	// For Router
	std::vector<std::pair<int, int>> two_pins_net;
	std::vector<Path*> paths;
	Net(){
		id = -1;
		this->pins.resize(0);
		this->vialist.clear();
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
		this->paths.resize(0);
	}
	Net(int _id) : id(_id){
		this->pins.resize(0);
		this->vialist.clear();
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
		this->paths.resize(0);
	}
	~Net(){
		for (auto p : paths) {
			delete p;
		}
	}

	int getWirelength(){
		// TODO compute the wirelength
		return 0;
	}
	// including adding via
	void segmentRegularize(){
		for(auto &p : this->paths){
			for(auto &s : p->segments){
				if(s->attribute == 0){
					this->horizontal_segments.emplace_back(s->x, s->y, s->attribute
						, s->neighbor, s->y, s->attribute);

					bool first_pin_is_via = true, second_pin_is_via = true;
					for(auto &pin : this->pins){
						if(pin == Coordinate3D(s->x, s->y, s->attribute)) first_pin_is_via = false;
						if(pin == Coordinate3D(s->neighbor, s->y, s->attribute)) second_pin_is_via = false;
					}
					if(first_pin_is_via) this->vialist.emplace(s->x, s->y);
					if(second_pin_is_via) this->vialist.emplace(s->neighbor, s->y);
					
				}
				else{
					this->vertical_segments.emplace_back(s->x, s->y, s->attribute
						, s->x, s->neighbor, s->attribute);

					bool first_pin_is_via = true, second_pin_is_via = true;
					for(auto &pin : this->pins){
						if(pin == Coordinate3D(s->x, s->y, s->attribute)) first_pin_is_via = false;
						if(pin == Coordinate3D(s->x, s->neighbor, s->attribute)) second_pin_is_via = false;
					}
					if(first_pin_is_via) this->vialist.emplace(s->x, s->y);
					if(second_pin_is_via) this->vialist.emplace(s->x, s->neighbor);
				}
			}
		}
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