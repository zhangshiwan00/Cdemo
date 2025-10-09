#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <cassert>

using namespace std;

const int MAX_NODES = 10000;
const int CHANNELS = 100;
const int INF = numeric_limits<int>::max();

struct Edge {
    int to;
    vector<int> channel_costs; // 100个通道的代价
    
    Edge(int t, const vector<int>& costs) : to(t), channel_costs(costs) {}
};

class ChannelGraph {
private:
    int node_count;
    vector<vector<Edge>> adj_list;
    vector<bool> node_support_convert;
    // 状态定义
    struct State {
        int cost;
        int node;
        int channel;        // 当前通道 (-1表示未开始)
        int consecutive;    // 连续通道计数
        int prev_node;      // 前驱节点
        int start_channel;  // 当前段的起始通道
        int business_width; // 业务宽度
        
        bool operator>(const State& other) const {
            return cost > other.cost;
        }
        
        State(int c, int n, int ch, int cons, int prev, int start, int bw) 
            : cost(c), node(n), channel(ch), consecutive(cons), 
              prev_node(prev), start_channel(start), business_width(bw) {}
    };
public:
    ChannelGraph(int n) : node_count(n), adj_list(n), node_support_convert(n, false) {}
    
    // 添加无向边
    void addEdge(int u, int v, const vector<int>& channel_costs) {
        if (u < 0 || u >= node_count || v < 0 || v >= node_count) {
            throw out_of_range("节点ID超出范围");
        }
        if (channel_costs.size() != CHANNELS) {
            throw invalid_argument("通道代价数组必须包含100个元素");
        }
        
        adj_list[u].emplace_back(v, channel_costs);
        adj_list[v].emplace_back(u, channel_costs);
    }
    
    // 设置节点是否支持通道转换
    void setNodeConversion(int node, bool support) {
        if (node < 0 || node >= node_count) {
            throw out_of_range("节点ID超出范围");
        }
        node_support_convert[node] = support;
    }
    
    // 寻找最短路径
    pair<vector<pair<int, int>>, int> findShortestPath(int source, int target, int channel_width) {
        // 输入验证
        if (channel_width < 1 || channel_width > 3) {
            throw invalid_argument("通道数量必须是1,2,3");
        }
        if (source < 0 || source >= node_count || target < 0 || target >= node_count) {
            throw out_of_range("节点ID超出范围");
        }
        
        // 距离数组: dist[node][start_channel] = 最小代价
        vector<vector<int>> dist(node_count, vector<int>(CHANNELS, INF));
        
        // 前驱节点: prev[node][start_channel] = (前驱节点, 前驱起始通道)
        vector<vector<pair<int, int>>> prev(node_count, vector<pair<int, int>>(CHANNELS, {-1, -1}));
        
        // 优先队列: (代价, 当前节点, 起始通道)
        using State = tuple<int, int, int>;
        priority_queue<State, vector<State>, greater<State>> pq;
        
        // 初始化源节点
        for (int start_ch = 0; start_ch <= CHANNELS - channel_width; ++start_ch) {
            dist[source][start_ch] = 0;
            pq.emplace(0, source, start_ch);
        }
        
        while (!pq.empty()) {
            auto [current_cost, u, u_start_ch] = pq.top();
            pq.pop();
            
            // 如果找到目标节点，重建路径
            if (u == target) {
                return reconstructPath(prev, source, target, u_start_ch, current_cost);
            }
            
            // 如果当前代价不是最小，跳过
            if (current_cost > dist[u][u_start_ch]) {
                continue;
            }
            
            // 遍历所有邻居
            for (const auto& edge : adj_list[u]) {
                int v = edge.to;
                
                // 确定可能的起始通道范围
                vector<int> possible_start_channels;
                if (node_support_convert[u] || u == source) {
                    // 支持转换或是源节点：可以任意选择起始通道
                    for (int ch = 0; ch <= CHANNELS - channel_width; ++ch) {
                        possible_start_channels.push_back(ch);
                    }
                } else {
                    // 不支持转换：必须使用相同起始通道
                    possible_start_channels.push_back(u_start_ch);
                }
                
                for (int v_start_ch : possible_start_channels) {
                    // 计算边(u,v)使用连续通道的代价
                    int channel_cost = calculateChannelCost(edge.channel_costs, v_start_ch, channel_width);
                    if (channel_cost == INF) continue;
                    
                    int new_cost = current_cost + channel_cost;
                    
                    // 更新距离
                    if (new_cost < dist[v][v_start_ch]) {
                        dist[v][v_start_ch] = new_cost;
                        prev[v][v_start_ch] = {u, u_start_ch};
                        pq.emplace(new_cost, v, v_start_ch);
                    }
                }
            }
        }
        
        return {vector<pair<int, int>>(), INF}; // 没有找到路径
    }

private:
    // 计算连续通道的代价
    int calculateChannelCost(const vector<int>& channel_costs, int start_ch, int width) {
        if (start_ch + width > CHANNELS) return INF;
        
        int total_cost = 0;
        for (int i = 0; i < width; ++i) {
            total_cost += channel_costs[start_ch + i];
        }
        return total_cost;
    }
    
    // 重建路径
    pair<vector<pair<int, int>>, int> reconstructPath(const vector<vector<pair<int, int>>>& prev, 
                                                     int source, int target, int target_ch, int cost) {
        vector<pair<int, int>> path;
        int current_node = target;
        int current_ch = target_ch;
        
        while (current_node != -1) {
            path.emplace_back(current_node, current_ch);
            auto [prev_node, prev_ch] = prev[current_node][current_ch];
            current_node = prev_node;
            current_ch = prev_ch;
        }
        
        reverse(path.begin(), path.end());
        return {path, cost};
    }
};

// 测试工具函数
class TestUtils {
public:
    static vector<int> generateChannelCosts(int base_cost = 1, int variation = 10) {
        vector<int> costs(CHANNELS);
        for (int i = 0; i < CHANNELS; ++i) {
            costs[i] = base_cost + (i % variation);
        }
        return costs;
    }
    
    static vector<int> generateConstantCosts(int cost = 1) {
        return vector<int>(CHANNELS, cost);
    }
};

// 测试用例
void runTests() {
    cout << "=== 测试通道约束最短路径算法 ===\n" << endl;
    
    // 测试用例1: 基本功能测试
    cout << "1. 基本功能测试" << endl;
    {
        ChannelGraph graph(6);
        
        // 构建简单图: 0-1-3-5 和 0-2-3-5
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(1));
        graph.addEdge(0, 2, TestUtils::generateConstantCosts(2));
        graph.addEdge(1, 3, TestUtils::generateConstantCosts(1));
        graph.addEdge(2, 3, TestUtils::generateConstantCosts(1));
        graph.addEdge(3, 5, TestUtils::generateConstantCosts(1));
        graph.addEdge(2, 4, TestUtils::generateConstantCosts(3));
        graph.addEdge(4, 5, TestUtils::generateConstantCosts(1));
        
        // 设置节点转换能力
        graph.setNodeConversion(0, true);
        graph.setNodeConversion(3, true);
        graph.setNodeConversion(5, false);
        
        auto [path, cost] = graph.findShortestPath(0, 5, 1);
        if (!path.empty()) {
            cout << "路径: ";
            for (const auto& [node, ch] : path) {
                cout << "(" << node << "," << ch << ") ";
            }
            cout << "\n总代价: " << cost << endl;
        } else {
            cout << "未找到路径" << endl;
        }
        cout << endl;
    }
    
    // 测试用例2: 不同通道宽度
    cout << "2. 通道宽度=2测试" << endl;
    {
        ChannelGraph graph(4);
        
        vector<int> costs1 = TestUtils::generateChannelCosts(1, 5);
        vector<int> costs2 = TestUtils::generateChannelCosts(2, 3);
        
        graph.addEdge(0, 1, costs1);
        graph.addEdge(1, 2, costs1);
        graph.addEdge(2, 3, costs2);
        
        graph.setNodeConversion(0, true);
        graph.setNodeConversion(1, false);
        graph.setNodeConversion(2, true);
        
        auto [path, cost] = graph.findShortestPath(0, 3, 2);
        if (!path.empty()) {
            cout << "路径长度: " << path.size() << ", 总代价: " << cost << endl;
        }
        cout << endl;
    }
    
    // 测试用例3: 无转换节点约束
    cout << "3. 无转换节点约束测试" << endl;
    {
        ChannelGraph graph(4);
        
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(1));
        graph.addEdge(1, 2, TestUtils::generateConstantCosts(1));
        graph.addEdge(2, 3, TestUtils::generateConstantCosts(1));
        
        // 所有节点都不支持转换
        for (int i = 0; i < 4; ++i) {
            graph.setNodeConversion(i, false);
        }
        
        auto [path, cost] = graph.findShortestPath(0, 3, 1);
        if (!path.empty()) {
            cout << "路径找到，代价: " << cost << endl;
        }
        cout << endl;
    }
    
    // 测试用例4: 不可达测试
    cout << "4. 不可达测试" << endl;
    {
        ChannelGraph graph(3);
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(1));
        
        auto [path, cost] = graph.findShortestPath(0, 2, 1);
        if (path.empty()) {
            cout << "正确: 未找到从0到2的路径" << endl;
        }
        cout << endl;
    }
    
    // 测试用例5: 单边测试
    cout << "5. 单边测试" << endl;
    {
        ChannelGraph graph(2);
        
        vector<int> costs(100);
        for (int i = 0; i < 100; ++i) {
            costs[i] = i + 1; // 通道代价递增: 1,2,3,...,100
        }
        
        graph.addEdge(0, 1, costs);
        graph.setNodeConversion(0, true);
        graph.setNodeConversion(1, false);
        
        auto [path, cost] = graph.findShortestPath(0, 1, 3);
        if (!path.empty()) {
            cout << "路径: ";
            for (const auto& [node, ch] : path) {
                cout << "(" << node << "," << ch << ") ";
            }
            cout << "\n总代价: " << cost << " (应该是最小的3个连续通道代价: 1+2+3=6)" << endl;
        }
        cout << endl;
    }
    
    // 测试用例6: 性能测试准备
    cout << "6. 性能测试准备" << endl;
    {
        const int N = 1000; // 使用1000节点进行性能测试
        ChannelGraph graph(N);
        
        // 创建链状图
        for (int i = 0; i < N - 1; ++i) {
            graph.addEdge(i, i + 1, TestUtils::generateConstantCosts(1));
            graph.setNodeConversion(i, i % 2 == 0); // 交替设置转换能力
        }
        
        cout << "性能测试图创建完成 (" << N << "个节点)" << endl;
        
        // 实际性能测试可以在需要时进行
        auto [path, cost] = graph.findShortestPath(0, N-1, 1);
        if (!path.empty()) {
            cout << "路径: ";
            for (const auto& [node, ch] : path) {
                cout << "(" << node << "," << ch << ") ";
            }
            cout << "\n总代价: " << cost << " (应该是999)" << endl;
        }
        cout << endl;
    }
}

int main() {
    try {
        runTests();
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
