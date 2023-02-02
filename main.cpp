#include <iostream>
#include "layout.hpp"
#include "io.hpp"
#include "timer.hpp"
int main(int argc, char const *argv[]){
    Timer timer;

    Layout layout;

    std::cout << "Reading Layout\n"; timer.setShortTerm();
    io::readLayout(&layout, argv[1]);
    std::cout << "Read time: " << timer.getShortTerm() << "\n";
    
    for(auto &n : layout.netlist) n.rmst_kruskal(layout.via_cost, layout.horizontal_segment_cost, layout.vertical_segment_cost);


    std::cout << "Write Layout\n"; timer.setShortTerm();
    io::writeLayout(&layout, argv[2]);
    std::cout << "Write time: " << timer.getShortTerm() << "\n";
    
    std::cout << "Tottal time: " << timer.getAndReset() << "\n";
    return 0;
}
