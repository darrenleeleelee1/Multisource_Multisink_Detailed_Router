#include "io.hpp"
int main(int argc, char const *argv[]){
    Layout layout;
    io::readLayout(&layout, argv[1]);
    io::writeLayout(&layout, argv[2]);
    return 0;
}
