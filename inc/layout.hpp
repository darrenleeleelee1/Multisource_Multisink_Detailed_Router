#pragma once
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
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
}
class Coordinate2D{
public:
	int x, y;
	Coordinate2D(){}
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
	Segment(int _z, int _x, int _y, int nei) : z(_z){
		if(_z == 0){
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
	int getX(){
		if(this->z == 0) return std::min(this->x, this->neighbor);
		else return this->x;
	}
	int getY(){
		if(this->z == 1) return std::min(this->y, this->neighbor);
		else return this->y;
	}
	int getNeighbor(){
		if(this->z == 0) return std::max(this->x, this->neighbor);
		else if(this->z == 1) return std::max(this->y, this->neighbor);
		else return this->neighbor;
	}
	Coordinate3D startPoint(){
		if(this->z == 0) return Coordinate3D{this->getX(), this->getY(), this->z};
		else return Coordinate3D{this->getX(), this->getY(), this->z};
	}
	Coordinate3D endPoint(){
		if(this->z == 0) return Coordinate3D{this->getNeighbor(), this->getY(), this->z};
		else return Coordinate3D{this->getX(), this->getNeighbor(), this->z};
	}
	std::string toString(){
		if(this->z == 0) {
			return "(" + std::to_string(this->getX()) + "," + std::to_string(this->getY()) + "," + std::to_string(this->z) + ")"
					+ "-(" + std::to_string(this->getNeighbor()) + "," + std::to_string(this->getY()) + "," + std::to_string(this->z) + ")";
		}
		else{
			return "(" + std::to_string(this->getX()) + "," + std::to_string(this->getY()) + "," + std::to_string(this->z) + ")"
					+ "-(" + std::to_string(this->getX()) + "," + std::to_string(this->getNeighbor()) + "," + std::to_string(this->z) + ")";
		}
	}
	bool colinear(Coordinate3D point){
		if(point.z != this->z) return false;
		if(this->z == 0){
			if(this->getY() != point.y) return false;
			if(this->getX() <= point.x && point.x <= this->getNeighbor()) return true;
		}
		else if(this->z == 1){
			if(this->getX() != point.x) return false;
			if(this->getY() <= point.y && point.y <= this->getNeighbor()) return true;
		}
		return false;
	}
	int getWirelength(){
		if(z == 0){
			return std::max(this->x, this->neighbor) - std::min(this->x, this->neighbor);
		}
		else if(z == 1){
			return std::max(this->y, this->neighbor) - std::min(this->y, this->neighbor);
		}
		return 0;
	}
};
class Edge{
public:
	std::vector<Segment*> segments;
	Coordinate3D start_pin, end_pin;
	Edge() {
		this->segments.resize(0);
	}
	Edge(const Edge& other) {
        this->segments.resize(0);
        for (auto s : other.segments) {
            this->segments.push_back(new Segment(*s));
        }
        this->start_pin = other.start_pin;
        this->end_pin = other.end_pin;
    }
	~Edge() {
		for (auto s : segments) {
			delete s;
		}
	}
};
class Subtree{
public:
    std::vector<Edge*> edges; // stores the edges in each subtree
	std::unordered_set<Coordinate3D> pinlist;
	
	Subtree() {}
	Subtree(Coordinate3D pin) {
		edges.resize(0);
		this->pinlist.insert(pin);
	}
	~Subtree(){
		for(auto e : edges){
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
	std::unordered_map<Coordinate3D, int> coordinate2index;
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
			this->coordinate2index[pins.at(i)] = i;
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
            // merge edges
            subtrees.at(x_root)->edges.insert(subtrees.at(x_root)->edges.end(), subtrees.at(y_root)->edges.begin(), subtrees.at(y_root)->edges.end());
            subtrees.at(y_root)->edges.clear();
			// merge pinlist
			subtrees.at(x_root)->pinlist.insert(subtrees.at(y_root)->pinlist.begin(), subtrees.at(y_root)->pinlist.end());
            subtrees.at(y_root)->pinlist.clear();
            return true;
        }
        return false;
    }
	std::pair<std::vector<Edge*>, std::vector<Edge*>> reconstructTree_phase1(){
		std::unordered_set<int> roots;
		// Store all the edges to new_edges
		std::vector<Edge*> all_edges;
		for(unsigned i = 0; i < coordinate2index.size(); i++){
			int root = find(i);
			if(!roots.count(root)){
				roots.insert(root);
				all_edges.insert(all_edges.end(), subtrees.at(root)->edges.begin(), subtrees.at(root)->edges.end());
			}
		}

		std::vector<Edge*> new_edges;
		for (auto edge : all_edges) {
			new_edges.push_back(new Edge(*edge));
		}

		return std::make_pair(all_edges, new_edges);
	}
	void reconstructTree_phase2(std::vector<Edge*> new_edges){
		for(unsigned i = 0; i < subtrees.size(); i++){
			delete subtrees.at(i);
		}
		// Reinitilize parents, size, subtree, coordinate2index
		unsigned n = pinset.size();
		parents.assign(n, -1);
        size.assign(n, 1);
		subtrees.resize(n);
		int cnt = 0;
		for (auto p : pinset){
			subtrees.at(cnt) = (new Subtree(p));
			coordinate2index[p] = cnt;
			cnt++;
		}
		for(auto e : new_edges){
			if(!coordinate2index.count(e->start_pin)){
				parents.push_back(-1);
				size.push_back(1);
				subtrees.push_back(new Subtree(e->start_pin));
				coordinate2index[e->start_pin] = cnt++;
			}
			if(!coordinate2index.count(e->end_pin)){
				parents.push_back(-1);
				size.push_back(1);
				subtrees.push_back(new Subtree(e->end_pin));
				coordinate2index[e->end_pin] = cnt++;
			}
		}
		for(auto e : new_edges){
			subtrees.at(coordinate2index[e->start_pin])->edges.push_back(e);
			if(!mergeTree(coordinate2index[e->start_pin], coordinate2index[e->end_pin])){
				throw std::runtime_error("Failure: not sure what this behavior");
			}
		}
	}
	std::vector<Edge*> getEdge(){
		std::unordered_set<int> roots;
		std::vector<Edge*> all_edges;
		for(unsigned i = 0; i < coordinate2index.size(); i++){
			int root = find(i);
			if(!roots.count(root)){
				roots.insert(root);
				all_edges.insert(all_edges.end(), subtrees.at(root)->edges.begin(), subtrees.at(root)->edges.end());
				// break; // Assume correct, it will only have one root
			}
		}
		return all_edges;
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
	int getCost(){
		// TODO: caculate
		return 0;
	}
	int getWirelength(){
		int sum = 0;
		for(auto &p : tree->getEdge()){
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
		for(auto &p : tree->getEdge()){
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
	int via_cost = 1;
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