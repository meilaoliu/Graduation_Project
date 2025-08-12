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
    State* pre;
    double distance;
    State(Vertex* _node , State* _pre , double _distance)
    :node(_node),pre(_pre),distance(_distance){}

    // bool operator<(State* other){
    //     return distance > other->distance;
    // }

};


//欧氏距离
double euclidean_distance(Vertex* a , Vertex* b){
        return sqrt(pow((a->x - b->x),2)+pow((a->y - b->y),2));
}

//自定义比较函数 
struct Compare{
    bool operator()( State* a , State* b){
        return a->distance > b->distance;
    }
};

vector<Vertex*>  Dijkstra(const vector<vector<Vertex*>> &graph , Vertex* start ,Vertex* end){
   WeightGraph diagraph(graph);
    
   //存储从起点到每个节点的最短距离
   unordered_map<Vertex* , double> disto;
   //最小堆优先队列
   priority_queue<State* , vector<State*> , Compare> pq;

   //用于存储所有分配的 State*，以便最后释放内存
   vector<State*>  all_states;
   
   vector<Vertex*> result;


    //将起点加入优先队列
   auto start_node = new State(start , nullptr , 0);
   pq.push(start_node);
   all_states.push_back(start_node);
   State* final_state = nullptr;



   while(!pq.empty()){
        State* currentstate = pq.top();
        Vertex* currentnode = currentstate->node;

        pq.pop();

        if(disto.count(currentnode))
        continue;

        disto[currentnode] = currentstate->distance;

        if(currentnode == end) {
            final_state = currentstate;//记录末尾位置
            break;
        }

        for(auto node:diagraph.neighbors(currentnode)){
            if(!disto.count(node)){
                double new_distance = disto[currentnode]+euclidean_distance(currentnode ,node);
                auto new_state = new State(node , currentstate , new_distance);
                pq.push(new_state);
                all_states.push_back(new_state);
            }
        }
   }
   
   //加入终点
   if(final_state==nullptr) return vector<Vertex*>();

   State* tmp = final_state;
   result.push_back(tmp->node);

   while(tmp->pre!=nullptr){
     tmp=tmp->pre;
     result.push_back(tmp->node);
   }

   reverse(result.begin() , result.end());

   for(auto state:all_states) delete state;
   
   return result;

}