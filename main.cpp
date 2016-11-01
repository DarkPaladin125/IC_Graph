#include <iostream>
#include <string>

#include "Graph.hpp"

using namespace std;

int main() {

    graph<int> g = { {1, 2}, {1, 3} };

    for (auto it : g) {
        cout << it.first << " <-> " << it.second << endl;
    }

    return 0;

}