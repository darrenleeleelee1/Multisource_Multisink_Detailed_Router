#pragma once
#include "layout.hpp"
class Router
{
public:
    Layout *layout;
    Router() {}
    Router(Layout *l) {
        layout = l;
    }
    ~Router() {}

    void main();
    void twoPinNetDecomposition();
};
