#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace std;
//定义顶点类
struct Vertex
{
    int id;
    double x ,y;
    string name;

    Vertex(int _id , double _x ,double _y , string _name)
    :id(_id),x(_x),y(_y),name(_name){}

};


//示例输入
// vector<Vertex *> v = valsToVets(vector<int>{1, 3, 2, 5, 4});
// vector<vector<Vertex *>> edges = {{v[0], v[1]}, {v[0], v[3]}, {v[1], v[2]},
//                                   {v[2], v[3]}, {v[2], v[4]}, {v[3], v[4]}};
//构造函数将其转为邻接列表
// 邻接表 =
// 1: [3, 5]
// 3: [1, 2]  
// 2: [3, 5, 4]
// 5: [1, 2, 4]
// 4: [2, 5]

//定义有权无向图类
class WeightGraph{
    public:

    WeightGraph(const vector<vector<Vertex*>>& _Graph){
        for(auto &edge:_Graph){
            addVertex(edge[0]);
            addVertex(edge[1]);
            addEdge(edge[0] , edge[1]);
        }

    }

    //添加边
    void addEdge(Vertex* a , Vertex* b){
        if(!Graph.count(a) || !Graph.count(b) || a ==b ){
            throw invalid_argument("不存在顶点");
        }
        Graph[a].push_back(b);
        Graph[b].push_back(a);//由于是无向图，所以相当于双向
    }

    //添加顶点
    void addVertex(Vertex* vet){
        if(Graph.count(vet)){
            return; //不重复添加顶点
        }
        Graph[vet] = vector<Vertex*>();
    }
    
    //返回邻居节点
    vector<Vertex*> neighbors(Vertex* a){
        return Graph[a];
    }


    private:
    unordered_map<Vertex* ,vector<Vertex*>> Graph;
        
};

// 1. 简化 State 结构体，不再需要 pre 指针
struct State {
    Vertex* node;
    double distance;

    State(Vertex* _node, double _distance) : node(_node), distance(_distance) {}

    // 定义比较运算符，方便优先队列使用
    bool operator>(const State& other) const {
        return distance > other.distance;
    }
};

// 独立的距离计算函数
double calculate_distance(Vertex* a, Vertex* b) {
    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2));
}

vector<Vertex*> Dijkstra_WithPredecessorMap(const vector<vector<Vertex*>>& graph, Vertex* start, Vertex* end) {
    WeightGraph diagraph(graph);
    
    // 存储从起点到每个节点的最短距离
    unordered_map<Vertex*, double> disto;
    // 2. 核心改动：使用 map 来记录每个节点的前驱节点
    unordered_map<Vertex*, Vertex*> predecessor;

    // 优先队列使用简化的 State，并利用其 `operator>` 变成最小堆
    priority_queue<State, vector<State>, greater<State>> pq;

    // 初始化
    disto[start] = 0;
    pq.emplace(start, 0);

    while (!pq.empty()) {
        State current_state = pq.top();
        pq.pop();

        Vertex* current_node = current_state.node;

        // 如果当前取出的距离比已经记录的还要长，说明是旧的、多余的路径，跳过
        if (current_state.distance > disto[current_node]) {
            continue;
        }

        // 如果到达终点，可以提前结束
        if (current_node == end) {
            break;
        }

        for (auto neighbor : diagraph.neighbors(current_node)) {
            double new_dist = disto[current_node] + calculate_distance(current_node, neighbor);
            
            // 如果找到了更短的路径 (或者第一次到达该邻居)
            if (!disto.count(neighbor) || new_dist < disto[neighbor]) {
                disto[neighbor] = new_dist;     // 更新最短距离
                pq.emplace(neighbor, new_dist); // 将新路径放入队列
                predecessor[neighbor] = current_node; // 3. 记录！“要到 neighbor，请从 current_node 来”
            }
        }
    }

    // --- 4. 路径回溯 ---
    vector<Vertex*> result;
    // 检查是否找到了通往终点的路径
    if (disto.count(end)) {
        Vertex* step = end;
        while (step != nullptr) {
            result.push_back(step);
            // 通过查询 map 回到上一步，如果查不到(说明到了起点)，step 会变成 nullptr
            step = predecessor.count(step) ? predecessor[step] : nullptr;
        }
        // 因为是从终点倒推的，所以需要反转得到正确顺序
        reverse(result.begin(), result.end());
    }

    return result;
}