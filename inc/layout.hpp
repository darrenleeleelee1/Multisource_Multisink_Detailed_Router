#pragma once
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
class Tree;
class Net;
class Coordinate3D{
public:
	int x, y, z;
	Coordinate3D(){}
	Coordinate3D(int _x, int  _y, int  _z) : x(_x), y(_y), z(_z) {}
	~Coordinate3D(){}
	std::string toString(){
		std::string tmp = "";
		tmp += "(" + std::to_string(this->x) + "," + std::to_string(this->y) + "," + std::to_string(this->z) + ")";
		return tmp;
	}
	bool operator==(const Coordinate3D &other) const {
    	return this->x == other.x && this->y == other.y && this->z == other.z;
    }
};
inline void hash_combine(std::size_t& seed, const int& v) {
	seed ^= std::hash<int>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
class Coordinate2D{
public:
	int x, y;
	Coordinate2D(){}
	Coordinate2D(Coordinate3D coor) : x(coor.x), y(coor.y) {} 
	Coordinate2D(int _x, int  _y) : x(_x), y(_y) {}
	~Coordinate2D(){}
	std::string toString(){
		std::string tmp = "";
		tmp += "(" + std::to_string(this->x) + "," + std::to_string(this->y) + ")";
		return tmp;
	}
	bool operator==(const Coordinate2D &other) const {
    	return this->x == other.x && this->y == other.y;
    }
	bool operator<(const Coordinate2D &other) const {
    	if (this->x == other.x)
        	return this->y < other.y;
        return this->x < other.x;
    }
};
namespace std {
    template<> struct hash<Coordinate3D> {
        size_t operator()(const Coordinate3D& c) const {
            size_t seed = 0;
            hash_combine(seed, c.x);
            hash_combine(seed, c.y);
            hash_combine(seed, c.z);
            return seed;
        }
    };

	template<> struct hash<Coordinate2D> {
        size_t operator()(const Coordinate2D& c) const {
            size_t seed = 0;
            hash_combine(seed, c.x);
            hash_combine(seed, c.y);
            return seed;
        }
    };
}
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
	int x, y, neighbor;
	int z; // 0 for horizon, 1 for verticle
	Segment(){}
	Segment(int _z, int _x, int _y, int nei) {
		this->z = _z;
		if(getLayer() == 0){
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
	int getLayer(){
		return this->z % 2;
	}
	int getX(){
		if(getLayer() == 0) return std::min(x, neighbor);
		else return x;
	}
	int getY(){
		if(getLayer() == 1) return std::min(y, neighbor);
		else return y;
	}
	int getNeighbor(){
		if(getLayer() == 0) return std::max(x, neighbor);
		else return std::max(y, neighbor);
	}
	Coordinate3D startPoint(){
		if(getLayer() == 0) return Coordinate3D{getX(), getY(), z};
		else return Coordinate3D{getX(), getY(), z};
	}
	Coordinate3D endPoint(){
		if(getLayer() == 0) return Coordinate3D{getNeighbor(), getY(), z};
		else return Coordinate3D{getX(), getNeighbor(), z};
	}
	std::string toString(){
		if(getLayer() == 0) {
			return "(" + std::to_string(getX()) + "," + std::to_string(getY()) + "," + std::to_string(z) + ")"
					+ "-(" + std::to_string(getNeighbor()) + "," + std::to_string(getY()) + "," + std::to_string(z) + ")";
		}
		else{
			return "(" + std::to_string(getX()) + "," + std::to_string(getY()) + "," + std::to_string(z) + ")"
					+ "-(" + std::to_string(getX()) + "," + std::to_string(getNeighbor()) + "," + std::to_string(z) + ")";
		}
	}
	bool colinear(Coordinate3D point){
		if(getLayer() == 0){
			if(getY() != point.y) return false;
			if(getX() <= point.x && point.x <= getNeighbor()) return true;
		}
		else{
			if(getX() != point.x) return false;
			if(getY() <= point.y && point.y <= getNeighbor()) return true;
		}
		return false;
	}
	int getWirelength(){
		if(getLayer() == 0){
			return std::max(x, neighbor) - std::min(x, neighbor);
		}
		else{
			return std::max(y, neighbor) - std::min(y, neighbor);
		}
	}
	double getCost(double horizontal_cost, double vertical_cost){
		if(getLayer() == 0){
			return (std::max(x, neighbor) - std::min(x, neighbor)) * horizontal_cost;
		}
		else{
			return (std::max(y, neighbor) - std::min(y, neighbor)) * vertical_cost;
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
	Path(const Path& other) {
        this->segments.resize(0);
        for (auto s : other.segments) {
            this->segments.push_back(new Segment(*s));
        }
        this->start_pin = other.start_pin;
        this->end_pin = other.end_pin;
    }
	~Path() {
		for (auto s : segments) {
			delete s;
		}
	}
	void lineUpSegments(){
		Coordinate3D sp = start_pin;
		unsigned swap_index;
		for(unsigned i = 0; i < segments.size(); i++){
			swap_index = i;
			for(unsigned j = i; j < segments.size(); j++){
				if(Coordinate2D{sp} == Coordinate2D{segments.at(j)->startPoint()} || Coordinate2D{sp} == Coordinate2D{segments.at(j)->endPoint()}){
					swap_index = j;
					break;
				}
			}
			std::swap(segments.at(i), segments.at(swap_index));
			auto &s = segments.at(i);
			sp = ((Coordinate2D{s->startPoint()} == Coordinate2D{sp}) ? Coordinate3D(s->endPoint().x, s->endPoint().y, (s->endPoint().z + 1) % 2) 
                                    : Coordinate3D(s->startPoint().x, s->startPoint().y, (s->startPoint().z + 1) % 2));
		}
	}
};
class Subtree{
public:
    std::vector<Path*> paths; // stores the paths in each subtree
	std::unordered_set<Coordinate3D> pinlist;
	
	Subtree() {}
	Subtree(Coordinate3D pin) {
		paths.resize(0);
		this->pinlist.insert(pin);
	}
	~Subtree(){
		for(auto e : paths){
			delete e;
		}
	}
	std::string showPins(){
		std::string tmp;
		for(auto p : pinlist) tmp += p.toString() + ", ";
		return tmp;
	}
};
class Tree{
public:
	std::vector<int> parents; // stores the parent of each node in the subtree
    std::vector<int> size; // stores the size of each subtree
	std::vector<Subtree*> subtrees;
	std::unordered_map<Coordinate2D, int> coordinate2index;
	std::unordered_set<Coordinate3D> pinset;
	Tree() {}
	Tree(std::vector<Coordinate3D> pins) {
		pinset.insert(pins.begin(), pins.end());
		unsigned n = pins.size();
		this->parents.resize(n, -1);
        this->size.resize(n, 1);
		subtrees.resize(n);
		for (unsigned i = 0; i < n; i++){
			subtrees.at(i) = (new Subtree(pins.at(i)));
			this->coordinate2index[Coordinate2D{pins.at(i)}] = i;
		}
	}
	Subtree* at(int index){
		return subtrees.at(find(index));
	}
	~Tree(){
		for(unsigned i = 0; i < subtrees.size(); i++){
			delete subtrees.at(i);
		}
	}
	int find(int x) {
		while (parents.at(x) >= 0) x = parents.at(x);
		return x;
	}
	bool mergeTree(int x, int y) {
        int x_root = find(x);
        int y_root = find(y);
        if (x_root != y_root) {
            if (size.at(x_root) < size.at(y_root)) {
                std::swap(x_root, y_root);
            }
            parents.at(y_root) = x_root;
            size.at(x_root) += size.at(y_root);
            // merge paths
            subtrees.at(x_root)->paths.insert(subtrees.at(x_root)->paths.end(), subtrees.at(y_root)->paths.begin(), subtrees.at(y_root)->paths.end());
            subtrees.at(y_root)->paths.clear();
			// merge pinlist
			subtrees.at(x_root)->pinlist.insert(subtrees.at(y_root)->pinlist.begin(), subtrees.at(y_root)->pinlist.end());
            subtrees.at(y_root)->pinlist.clear();
            return true;
        }
        return false;
    }
	std::pair<std::vector<Path*>, std::vector<Path*>> reconstructTree_phase1(){
		std::unordered_set<int> roots;
		// Store all the paths to new_paths
		std::vector<Path*> all_paths;
		for(unsigned i = 0; i < coordinate2index.size(); i++){
			int root = find(i);
			if(!roots.count(root)){
				roots.insert(root);
				all_paths.insert(all_paths.end(), subtrees.at(root)->paths.begin(), subtrees.at(root)->paths.end());
			}
		}

		std::vector<Path*> new_paths;
		for (auto path : all_paths) {
			new_paths.push_back(new Path(*path));
		}

		return std::make_pair(all_paths, new_paths);
	}
	void reconstructTree_phase2(std::vector<Path*> new_paths){
		for(unsigned i = 0; i < subtrees.size(); i++){
			delete subtrees.at(i);
		}
		// Reinitilize parents, size, subtree, coordinate2index
		unsigned n = pinset.size();
		coordinate2index.clear();
		parents.assign(n, -1);
        size.assign(n, 1);
		subtrees.resize(n);
		int cnt = 0;
		for (auto p : pinset){
			subtrees.at(cnt) = (new Subtree(p));
			coordinate2index[Coordinate2D{p}] = cnt;
			cnt++;
		}
		for(auto e : new_paths){
			if(!coordinate2index.count(Coordinate2D{e->start_pin})){
				parents.push_back(-1);
				size.push_back(1);
				subtrees.push_back(new Subtree(e->start_pin));
				coordinate2index[Coordinate2D{e->start_pin}] = cnt++;
			}
			if(!coordinate2index.count(Coordinate2D{e->end_pin})){
				parents.push_back(-1);
				size.push_back(1);
				subtrees.push_back(new Subtree(e->end_pin));
				coordinate2index[Coordinate2D{e->end_pin}] = cnt++;
			}
		}
		for(auto e : new_paths){
			int root = find(coordinate2index[Coordinate2D{e->start_pin}]);
			subtrees.at(root)->paths.push_back(e);
			if(!mergeTree(coordinate2index[Coordinate2D{e->start_pin}], coordinate2index[Coordinate2D{e->end_pin}])){
				throw std::runtime_error("Failure: not sure what this behavior");
			}
		}
	}
	std::vector<Path*> getPath(){
		std::unordered_set<int> roots;
		std::vector<Path*> all_paths;
		for(unsigned i = 0; i < coordinate2index.size(); i++){
			int root = find(i);
			if(!roots.count(root)){
				roots.insert(root);
				all_paths.insert(all_paths.end(), subtrees.at(root)->paths.begin(), subtrees.at(root)->paths.end());
				// break; // Assume correct, it will only have one root
			}
		}
		return all_paths;
	}
};
class Net{
public:
	// For Input
	int id;
	std::vector<Coordinate3D> pins;
	// For Ouput
	std::set<Coordinate2D> vialist;
	// Segment for drawing
	std::vector<Segment_Draw> horizontal_segments;
	std::vector<Segment_Draw> vertical_segments;
	// For Router
	std::vector<std::pair<int, int>> two_pins_net;
	Tree* tree = nullptr;
	Net(){
		id = -1;
		this->pins.resize(0);
		this->vialist.clear();
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
	}
	Net(int _id) : id(_id){
		this->pins.resize(0);
		this->vialist.clear();
		this->horizontal_segments.resize(0);
		this->vertical_segments.resize(0);
	}
	~Net(){
		delete tree;
	}
	void initTrees(){
		tree = new Tree(pins);
	}
	double getCost(double horizontal_cost, double vertical_cost, double via_cost){
		double sum = 0;
		for(auto &p : tree->getPath()){
			for(auto &s : p->segments){
				sum += s->getCost(horizontal_cost, vertical_cost);
			}
		}
		sum += vialist.size() * via_cost;
		return sum;
	}
	int getWirelength(){
		int sum = 0;
		for(auto &p : tree->getPath()){
			for(auto &s : p->segments){
				sum += s->getWirelength();
			}
		}
		return sum;
	}
	bool checkIsPin(Coordinate3D target){
		for(auto p : pins){
			if(p == target) return true;
		}
		return false;
	}
	// including adding via
	void segmentRegularize(){
		for(auto &p : tree->getPath()){
			if(p->segments.size() == 0){
				if(p->start_pin == p->end_pin){
					this->vialist.emplace(p->start_pin.x, p->start_pin.y);
				}
			}
			else{
				for(auto &s : p->segments){
					if(s->z == 0){
						this->horizontal_segments.emplace_back(s->x, s->y, s->z
							, s->neighbor, s->y, s->z);

						bool first_pin_is_via = true, second_pin_is_via = true;
						for(auto &pin : this->pins){
							if(pin == Coordinate3D(s->x, s->y, s->z)) first_pin_is_via = false;
							if(pin == Coordinate3D(s->neighbor, s->y, s->z)) second_pin_is_via = false;
						}
						if(first_pin_is_via) this->vialist.emplace(s->x, s->y);
						if(second_pin_is_via) this->vialist.emplace(s->neighbor, s->y);
						
					}
					else{
						this->vertical_segments.emplace_back(s->x, s->y, s->z
							, s->x, s->neighbor, s->z);

						bool first_pin_is_via = true, second_pin_is_via = true;
						for(auto &pin : this->pins){
							if(pin == Coordinate3D(s->x, s->y, s->z)) first_pin_is_via = false;
							if(pin == Coordinate3D(s->x, s->neighbor, s->z)) second_pin_is_via = false;
						}
						if(first_pin_is_via) this->vialist.emplace(s->x, s->y);
						if(second_pin_is_via) this->vialist.emplace(s->x, s->neighbor);
					}
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
	double via_cost = 1;
	double horizontal_segment_cost = 1, vertical_segment_cost = 1;
	std::vector<Obstacle> obstacles;
	std::vector<Net> netlist;
	
	Layout(){}
	Layout(int w, int h, int nol, double vc, double hsc, double vsc)
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

	double getCost(){
		double sum = 0;
		for(unsigned i = 0; i < this->netlist.size(); i++){
			sum += this->netlist.at(i).getCost(horizontal_segment_cost, vertical_segment_cost, via_cost);
		}
		return sum;
	}

	// debug
	void writeForDebug(unsigned test_num){
		char file_path[64]; 
		sprintf(file_path, "out/tmp_%u.txt", test_num);
		std::ofstream out_file(file_path, std::ofstream::trunc);
		out_file << "Width " << this->width << "\n";
		out_file << "Height " << this->height << "\n";
		out_file << "Layer " << this->num_of_layers << "\n";
		out_file << "Total_WL " << 0 << "\n";
		out_file << "Cost " << 0 << "\n";
		out_file << "Obstacle_num " << this->obstacles.size() << "\n";
		for(unsigned i = 0; i < this->obstacles.size(); i++){
			out_file << this->obstacles.at(i).start_point.x << " " << this->obstacles.at(i).start_point.y << " " << this->obstacles.at(i).start_point.z << " ";
			if(this->obstacles.at(i).start_point.z == 0)
				out_file << this->obstacles.at(i).end_point.x << " " << this->obstacles.at(i).end_point.y + 1<< " " << this->obstacles.at(i).end_point.z;
			else
				out_file << this->obstacles.at(i).end_point.x + 1 << " " << this->obstacles.at(i).end_point.y<< " " << this->obstacles.at(i).end_point.z;
			out_file << "\n";
		}
		out_file << "Net_num " << 1 << "\n";
		for(unsigned i = test_num; i <= test_num; i++){
			// debug
			this->netlist.at(i).horizontal_segments.clear();
			this->netlist.at(i).vertical_segments.clear();
			this->netlist.at(i).vialist.clear();
			// debug
			this->netlist.at(i).segmentRegularize();
			out_file << "Net_id " << this->netlist.at(i).id << "\n";
			out_file << "pin_num " << this->netlist.at(i).pins.size() << "\n";
			for(unsigned j = 0; j < this->netlist.at(i).pins.size(); j++){
				out_file << this->netlist.at(i).pins.at(j).x << " " << this->netlist.at(i).pins.at(j).y << " " << this->netlist.at(i).pins.at(j).z << "\n";
			}
			out_file << "Via_num " << this->netlist.at(i).vialist.size() << "\n";
			for(auto &v : this->netlist.at(i).vialist){
				out_file << v.x << " " << v.y << "\n";
			}
			out_file << "H_segment_num " << this->netlist.at(i).horizontal_segments.size() << "\n";
			for(unsigned j = 0; j < this->netlist.at(i).horizontal_segments.size(); j++){
				out_file << this->netlist.at(i).horizontal_segments.at(j).start_point.x << " " << this->netlist.at(i).horizontal_segments.at(j).start_point.y 
						<< " " << this->netlist.at(i).horizontal_segments.at(j).start_point.z << " ";
				out_file << this->netlist.at(i).horizontal_segments.at(j).end_point.x + 1 << " " << this->netlist.at(i).horizontal_segments.at(j).end_point.y + 1
						<< " " << this->netlist.at(i).horizontal_segments.at(j).end_point.z << "\n";
			}
			out_file << "V_segment_num " << this->netlist.at(i).vertical_segments.size() << "\n";
			for(unsigned j = 0; j < this->netlist.at(i).vertical_segments.size(); j++){
				out_file << this->netlist.at(i).vertical_segments.at(j).start_point.x << " " << this->netlist.at(i).vertical_segments.at(j).start_point.y 
						<< " " << this->netlist.at(i).vertical_segments.at(j).start_point.z << " ";
				out_file << this->netlist.at(i).vertical_segments.at(j).end_point.x + 1 << " " << this->netlist.at(i).vertical_segments.at(j).end_point.y + 1
						<< " " << this->netlist.at(i).vertical_segments.at(j).end_point.z << "\n";
			}
		}
	}
	// debug
};