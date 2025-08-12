#include <string>
#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <memory> // 为智能指针添加头文件
using namespace std;

// 定义顶点类
struct Vertex {
    int id;
    double x, y;
    string name;

    Vertex(int _id, double _x, double _y, string _name)
        :id(_id), x(_x), y(_y), name(_name) {}
};

// 定义有权无向图类
class WeightGraph {
public:
    WeightGraph(const vector<vector<Vertex*>>& _Graph) {
        for (auto& edge : _Graph) {
            addVertex(edge[0]);
            addVertex(edge[1]);
            addEdge(edge[0], edge[1]);
        }
    }

    // 添加边
    void addEdge(Vertex* a, Vertex* b) {
        if (!Graph.count(a) || !Graph.count(b) || a == b) {
            throw invalid_argument("不存在顶点");
        }
        Graph[a].push_back(b);
        Graph[b].push_back(a); // 由于是无向图，所以相当于双向
    }

    // 添加顶点
    void addVertex(Vertex* vet) {
        if (Graph.count(vet)) {
            return; // 不重复添加顶点
        }
        Graph[vet] = vector<Vertex*>();
    }
    
    // 返回邻居节点
    vector<Vertex*> neighbors(Vertex* a) {
        return Graph[a];
    }

private:
    unordered_map<Vertex*, vector<Vertex*>> Graph;
};

// 使用智能指针的State结构体
struct State {
    Vertex* node;
    shared_ptr<State> pre; // 使用shared_ptr替代原始指针
    double distance;
    
    State(Vertex* _node, shared_ptr<State> _pre, double _distance)
        :node(_node), pre(_pre), distance(_distance) {}
};

// 欧氏距离
double euclidean_distance(Vertex* a, Vertex* b) {
    return sqrt(pow((a->x - b->x), 2) + pow((a->y - b->y), 2));
}

// 自定义比较函数，使用智能指针
struct Compare {
    bool operator()(const shared_ptr<State>& a, const shared_ptr<State>& b) {
        return a->distance > b->distance;
    }
};

vector<Vertex*> Dijkstra(const vector<vector<Vertex*>>& graph, Vertex* start, Vertex* end) {
    WeightGraph diagraph(graph);
    
    // 存储从起点到每个节点的最短距离
    unordered_map<Vertex*, double> disto;
    
    // 最小堆优先队列，使用智能指针
    priority_queue<shared_ptr<State>, vector<shared_ptr<State>>, Compare> pq;
    
    vector<Vertex*> result;

    // 将起点加入优先队列
    auto start_node = make_shared<State>(start, nullptr, 0);
    pq.push(start_node);
    
    shared_ptr<State> final_state = nullptr;

    while (!pq.empty()) {
        shared_ptr<State> currentstate = pq.top();
        Vertex* currentnode = currentstate->node;
        pq.pop();

        // 如果节点已处理，跳过
        if (disto.count(currentnode))
            continue;

        disto[currentnode] = currentstate->distance;

        // 找到终点，结束搜索
        if (currentnode == end) {
            final_state = currentstate; // 记录末尾位置
            break;
        }

        // 处理邻居节点
        for (auto node : diagraph.neighbors(currentnode)) {
            if (!disto.count(node)) {
                double new_distance = disto[currentnode] + euclidean_distance(currentnode, node);
                auto new_state = make_shared<State>(node, currentstate, new_distance);
                pq.push(new_state);
            }
        }
    }
   
    // 回溯路径
    if (final_state == nullptr) 
        return vector<Vertex*>();

    shared_ptr<State> tmp = final_state;
    result.push_back(tmp->node);

    while (tmp->pre != nullptr) {
        tmp = tmp->pre;
        result.push_back(tmp->node);
    }

    reverse(result.begin(), result.end());
    
    // 不需要手动释放内存，智能指针会自动处理
    return result;
}