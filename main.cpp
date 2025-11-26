#include <vector>
#include <fstream>
#include <iostream>

int const N = 1000;
std::vector<int> G[N];

struct{
    int x, y;
}Point[N];

int main(){
    std::ifstream fin("data.in");

    int n;
    fin >> n;
    for(int i = 1; i <= n; ++i)
        for(int j = 1; j <= n; ++j)
            {
                bool x;
                fin >> x;
                if(x == 1){
                    G[i].push_back(j);
                    G[j].push_back(i);
                }

            }
    for(int i = 1; i <= n; ++i)
        Point[i].x = i, Point[i].y = (i*i) % n;
}