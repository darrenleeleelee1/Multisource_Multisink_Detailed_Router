#include <algorithm>
#include <iostream>
#include "layout.hpp"
class Kruskal{
public:
    class Edge{
    public:
        int from, to; // In this case is undirected
        int cost;
        Edge(){}
        Edge(int f, int t, int c) : from(f), to(t), cost(c) {}
        bool operator<(const Edge other){
            return cost < other.cost;
        }
    };
    Net *net;
    std::vector<int> parents;
    std::vector<Kruskal::Edge> edges;
    Kruskal(){}
    Kruskal(Net *n, int via_cost, int horizontal_segments_cost, int vertical_segment_cost){
        net = n;
        // set every pins parent as -1
        this->parents.assign(n->pins.size(), -1);
        // build the complete graph by every node rectilinear distance, scaled with the hsegment/vsegment cost and via cost
        for(unsigned i = 0; i < n->pins.size(); i++){
            for(unsigned j = i + 1; j < n->pins.size(); j++){
                int x_segment, y_segment;
                int sum_cost = 0;
                x_segment = std::max(n->pins.at(i).x - n->pins.at(j).x, n->pins.at(j).x - n->pins.at(i).x)
                                    - std::min(n->pins.at(i).x - n->pins.at(j).x, n->pins.at(j).x - n->pins.at(i).x);
                y_segment = std::max(n->pins.at(i).y - n->pins.at(j).y, n->pins.at(j).y - n->pins.at(i).y)
                                    - std::min(n->pins.at(i).y - n->pins.at(j).y, n->pins.at(j).y - n->pins.at(i).y);
                if(y_segment != 0) sum_cost += 2 * via_cost;
                sum_cost += x_segment * horizontal_segments_cost + y_segment * vertical_segment_cost;
                this->edges.emplace_back(i, j, sum_cost);
            }
        }
    }

    int find(int x){
        return this->parents[x] < 0 ? x : (this->parents[x] = Kruskal::find(this->parents[x]));
    }
    bool uni(int x ,int y){
        int xRoot = Kruskal::find(x), yRoot = Kruskal::find(y);
        if(xRoot != yRoot){
            if(this->parents[xRoot] < this->parents[yRoot]){
                this->parents[xRoot] += this->parents[yRoot];
                this->parents[yRoot] = xRoot;
            }
            else{
                this->parents[yRoot] += this->parents[xRoot];
                this->parents[xRoot] = yRoot;	
            }
            return true;
        }
        else return false;
    }
    int cost_rmst(){
        std::sort(this->edges.begin(), this->edges.end());
        int ans = 0;
        unsigned edge_cnt = 0;
        for(unsigned i = 0; i < this->edges.size(); i++){
            if(Kruskal::uni(this->edges.at(i).from, this->edges.at(i).to)){
                ans += this->edges.at(i).cost;
                // store the two pin nets
                this->net->two_pins_net.push_back(std::make_pair(this->edges.at(i).from, this->edges.at(i).to));
                if(++edge_cnt == this->parents.size() - 1) break;
            }
        }
        if(edge_cnt == this->parents.size() - 1) return ans;
        else return -1;// means can't found spanning tree
    }
};
void Net::rmst_kruskal(int via_cost, int horizontal_segments_cost, int vertical_segment_cost){
	Kruskal kruskal(this, via_cost, horizontal_segments_cost, vertical_segment_cost);
    int cost_rmst = kruskal.cost_rmst();
    if(cost_rmst == -1) std::cout << "Error, net#" << this->id << " RMST build failed.\n";
}