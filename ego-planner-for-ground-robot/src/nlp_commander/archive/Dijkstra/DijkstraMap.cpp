#include <string>
#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
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

struct State
{
    Vertex* node;
    double distance;

     // 构造函数接
    State(Vertex* _node, double _distance)
        :node(_node), distance(_distance) {}

     // 保留原始构造函数，但标记为弃用（可选）
    State(Vertex* _node, State* _pre, double _distance)
        :node(_node), distance(_distance) {}
    // bool operator<(State* other){
    //     return distance > other->distance;
    // }W

};


//欧氏距离
double euclidean_distance(Vertex* a , Vertex* b){
        return sqrt(pow((a->x - b->x),2)+pow((a->y - b->y),2));
}

//自定义比较函数 
struct Compare{
    bool operator()( State& a , State& b){
        return a.distance > b.distance;
    }
};

vector<Vertex*>  Dijkstra(const vector<vector<Vertex*>> &graph , Vertex* start ,Vertex* end){
   WeightGraph diagraph(graph);
    
   //存储从起点到每个节点的最短距离
   unordered_map<Vertex* , double> disto;
   //最小堆优先队列
   priority_queue<State , vector<State> , Compare> pq;

   //存储前驱节点
   unordered_map<Vertex* ,Vertex*> pre;
   //存储结果
   vector<Vertex*> result;

    //将起点加入优先队列
   pq.emplace(start ,0);

   pre[start] = nullptr;

   while(!pq.empty() ){
        State currentstate = pq.top();
        Vertex* currentnode = currentstate.node;

        pq.pop();

        if(disto.count(currentnode))
        continue;

        disto[currentnode] = currentstate.distance;
        
        if(currentnode == end) {
            break;
        }

        for(auto node:diagraph.neighbors(currentnode)){
            if(!disto.count(node)){
                double new_distance = disto[currentnode]+euclidean_distance(currentnode ,node);
                pq.emplace(node , new_distance);
                pre[node] = currentnode;
            }
        }
   }
   
   //如果没有找到路径
   if(!disto.count(end)) return vector<Vertex*>();

   // 回溯路径
   Vertex* current = end;
   while(current!=nullptr){
     result.push_back(current);
     if(current == start) break;
     current = pre[current];
   }

   reverse(result.begin() , result.end());
   return result;

}