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
        this->parents.assign(n->pins.size(), -1);
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
                // For RMST visualize
                int from = this->edges.at(i).from, to = this->edges.at(i).to;
                int x_segment, y_segment;
                x_segment = std::max(this->net->pins.at(from).x - this->net->pins.at(to).x, this->net->pins.at(to).x - this->net->pins.at(from).x)
                                    - std::min(this->net->pins.at(from).x - this->net->pins.at(to).x, this->net->pins.at(to).x - this->net->pins.at(from).x);
                y_segment = std::max(this->net->pins.at(from).y - this->net->pins.at(to).y, this->net->pins.at(to).y - this->net->pins.at(from).y)
                                    - std::min(this->net->pins.at(from).y - this->net->pins.at(to).y, this->net->pins.at(to).y - this->net->pins.at(from).y);
                if(y_segment != 0){ 
                    if(this->net->pins.at(from).y > this->net->pins.at(to).y){
                        if(this->net->pins.at(from).x < this->net->pins.at(to).x)
                            this->net->vertical_segments.emplace_back(std::min(this->net->pins.at(from).x, this->net->pins.at(to).x), std::min(this->net->pins.at(from).y, this->net->pins.at(to).y), 1
                                                        , std::min(this->net->pins.at(from).x, this->net->pins.at(to).x), std::max(this->net->pins.at(from).y, this->net->pins.at(to).y), 1);
                        else{
                            this->net->vertical_segments.emplace_back(std::max(this->net->pins.at(from).x, this->net->pins.at(to).x), std::min(this->net->pins.at(from).y, this->net->pins.at(to).y), 1
                                                        , std::max(this->net->pins.at(from).x, this->net->pins.at(to).x), std::max(this->net->pins.at(from).y, this->net->pins.at(to).y), 1);
                        }
                    }
                    else{
                        if(this->net->pins.at(from).x < this->net->pins.at(to).x)
                            this->net->vertical_segments.emplace_back(std::max(this->net->pins.at(from).x, this->net->pins.at(to).x), std::min(this->net->pins.at(from).y, this->net->pins.at(to).y), 1
                                                        , std::max(this->net->pins.at(from).x, this->net->pins.at(to).x), std::max(this->net->pins.at(from).y, this->net->pins.at(to).y), 1);
                        else{
                            this->net->vertical_segments.emplace_back(std::min(this->net->pins.at(from).x, this->net->pins.at(to).x), std::min(this->net->pins.at(from).y, this->net->pins.at(to).y), 1
                                                        , std::min(this->net->pins.at(from).x, this->net->pins.at(to).x), std::max(this->net->pins.at(from).y, this->net->pins.at(to).y), 1);
                        }
                    }
                }
                if(x_segment != 0) 
                    this->net->horizontal_segments.emplace_back(std::min(this->net->pins.at(from).x, this->net->pins.at(to).x), std::min(this->net->pins.at(from).y, this->net->pins.at(to).y), 0
                                                        , std::max(this->net->pins.at(from).x, this->net->pins.at(to).x), std::min(this->net->pins.at(from).y, this->net->pins.at(to).y), 0);
                
                
                if(++edge_cnt == this->parents.size() - 1) break;
            }
        }
        if(edge_cnt == this->parents.size() - 1) return ans;
        else return -1;// means can't found spanning tree
    }
};
void Net::rmst_kruskal(int via_cost, int horizontal_segments_cost, int vertical_segment_cost){
	Kruskal kruskal(this, via_cost, horizontal_segments_cost, vertical_segment_cost);
    std::cout << "Net id#"<< this->id << " cost of rmst= " << kruskal.cost_rmst() << "\n";
}