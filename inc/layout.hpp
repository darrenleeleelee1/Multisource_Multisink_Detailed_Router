#include <vector>
class Coordinate{
public:
	int x, y, z;
	Coordinate(){}
	Coordinate(int _x,int  _y,int  _z) : x(_x), y(_y), z(_z) {}
	~Coordinate(){}
};
class Obstacle{
public:
	Coordinate start_point, end_point;
	Obstacle(){}
	Obstacle(int _spx, int  _spy, int  _spz, int _epx, int  _epy, int  _epz)
		: start_point(_spx, _spy, _spz), end_point(_epx, _epy, _epz) {}
	~Obstacle(){}
};
class Net{
public:
	int id;
	int num_of_pins;
	std::vector<Coordinate> pins;
	Net(){}
	Net(int _id, int nop) : id(_id), num_of_pins(nop) {}
	~Net(){}

	void addPin(int _x,int  _y,int  _z){
		this->pins.emplace_back(_x, _y, _z);
	}
	void addPin(Coordinate p){
		this->pins.push_back(p);
	}

};
class Layout{
public:
	int width, height;
	int num_of_layers, num_of_obstacles, num_of_nets;
	int via_cost = 1;
	int horizontal_segment_cost = 1, vertical_segment_cost = 1;
	std::vector<Obstacle> obstacles;
	std::vector<Net> netlist;
	Layout(){}
	Layout(int w, int h, int nol, int noo, int vc, int hsc, int vsc)
		: width(w), height(h), num_of_layers(nol), num_of_obstacles(noo), via_cost(vc)
			, horizontal_segment_cost(hsc), vertical_segment_cost(vsc) {}
	~Layout(){}
	
	void addObstacle(int _spx, int  _spy, int  _spz, int _epx, int  _epy, int  _epz){
		this->obstacles.emplace_back(_spx, _spy, _spz, _epx, _epy, _epz);
	}
	void addObstacle(Obstacle o){
		this->obstacles.push_back(o);
	}

	void addNet(Net n){
		this->netlist.push_back(n);
	}
};