#include <iostream>
#include <vector>
#include <string>
#include "DijkstraMap.cpp"  // 包含你的实现

// 打印路径
void printPath(const vector<Vertex*>& path) {
    if (path.empty()) {
        cout << "无路径" << endl;
        return;
    }
    
    cout << "路径: ";
    for (size_t i = 0; i < path.size(); ++i) {
        cout << path[i]->name;
        if (i < path.size() - 1) {
            cout << " -> ";
        }
    }
    cout << endl;
    
    // 打印总距离
    double totalDist = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        totalDist += euclidean_distance(path[i], path[i+1]);
    }
    cout << "总距离: " << totalDist << endl;
}

int main() {
    // 创建顶点
    Vertex* a = new Vertex(1, 0, 0, "A");
    Vertex* b = new Vertex(2, 1, 1, "B");
    Vertex* c = new Vertex(3, 2, 0, "C");
    Vertex* d = new Vertex(4, 0, 2, "D");
    Vertex* e = new Vertex(5, 3, 2, "E");
    
    // 存储所有顶点以便最后释放内存
    vector<Vertex*> vertices = {a, b, c, d, e};
    
    // 创建边
    vector<vector<Vertex*>> edges = {
        {a, b}, {a, d},  // A连接到B和D
        {b, c}, {b, d},  // B连接到C和D
        {c, e},          // C连接到E
        {d, e}           // D连接到E
    };
    
    // 测试用例1: A到E的最短路径
    cout << "测试1: A到E的最短路径" << endl;
    vector<Vertex*> path1 = Dijkstra(edges, a, e);
    printPath(path1);
    cout << endl;
    
    // 测试用例2: C到A的最短路径
    cout << "测试2: C到A的最短路径" << endl;
    vector<Vertex*> path2 = Dijkstra(edges, c, a);
    printPath(path2);
    cout << endl;
    
    // 测试用例3: A到不存在的顶点的路径
    Vertex* f = new Vertex(6, 10, 10, "F");  // 不连接到任何顶点
    vertices.push_back(f);
    
    cout << "测试3: A到F的路径(应该不存在)" << endl;
    vector<Vertex*> path3 = Dijkstra(edges, a, f);
    printPath(path3);
    
    // 释放内存
    for (auto v : vertices) {
        delete v;
    }
    
    return 0;
}