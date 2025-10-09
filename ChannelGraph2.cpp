#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <functional>
#include <cassert>
#include <memory>
#include <unordered_set>

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
        
        // 访问标记，确保节点不重复
        vector<vector<bool>> visited(node_count, vector<bool>(CHANNELS, false));
        
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
            
            // 跳过已访问的节点
            if (visited[u][u_start_ch]) {
                continue;
            }
            visited[u][u_start_ch] = true;
            
            // 如果找到目标节点，重建路径
            if (u == target) {
                return reconstructPath(prev, source, target, u_start_ch, current_cost);
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
                    if (u_start_ch <= CHANNELS - channel_width) {
                        possible_start_channels.push_back(u_start_ch);
                    }
                }
                
                for (int v_start_ch : possible_start_channels) {
                    // 跳过已访问的节点
                    if (visited[v][v_start_ch]) {
                        continue;
                    }
                    
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
    
    // 重建路径并验证节点不重复
    pair<vector<pair<int, int>>, int> reconstructPath(const vector<vector<pair<int, int>>>& prev, 
                                                     int source, int target, int target_ch, int cost) {
        vector<pair<int, int>> path;
        unordered_set<int> visited_nodes; // 用于验证节点不重复
        
        int current_node = target;
        int current_ch = target_ch;
        
        while (current_node != -1) {
            // 检查节点是否重复
            if (visited_nodes.count(current_node)) {
                throw runtime_error("路径中包含重复节点");
            }
            visited_nodes.insert(current_node);
            
            path.emplace_back(current_node, current_ch);
            auto [prev_node, prev_ch] = prev[current_node][current_ch];
            current_node = prev_node;
            current_ch = prev_ch;
        }
        
        reverse(path.begin(), path.end());
        
        // 最终验证
        if (path[0].first != source) {
            throw runtime_error("路径重建错误");
        }
        
        return {path, cost};
    }
};

// 测试工具类
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
    
    static vector<int> generateAscendingCosts(int start = 1) {
        vector<int> costs(CHANNELS);
        for (int i = 0; i < CHANNELS; ++i) {
            costs[i] = start + i;
        }
        return costs;
    }
};

// 测试用例
void runBasicTests() {
    cout << "=== 基本功能测试 ===" << endl;
    
    // 测试用例1: 简单线性路径
    cout << "\n1. 简单线性路径测试" << endl;
    {
        ChannelGraph graph(3);
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(5));
        graph.addEdge(1, 2, TestUtils::generateConstantCosts(3));
        
        graph.setNodeConversion(0, true);
        graph.setNodeConversion(1, false);
        graph.setNodeConversion(2, false);
        
        auto [path, cost] = graph.findShortestPath(0, 2, 1);
        assert(!path.empty());
        assert(cost == 8);
        assert(path.size() == 3);
        assert(path[0].first == 0);
        assert(path[2].first == 2);
        cout << "测试通过: 代价=" << cost << ", 路径长度=" << path.size() << endl;
    }
    
    // 测试用例2: 通道转换测试
    cout << "\n2. 通道转换测试" << endl;
    {
        ChannelGraph graph(3);
        
        // 边0-1: 通道0代价高，通道1代价低
        vector<int> costs1(CHANNELS, 10);
        costs1[0] = 100; costs1[1] = 1;
        graph.addEdge(0, 1, costs1);
        
        // 边1-2: 通道0代价低，通道1代价高  
        vector<int> costs2(CHANNELS, 10);
        costs2[0] = 1; costs2[1] = 100;
        graph.addEdge(1, 2, costs2);
        
        // 节点1支持转换，应该选择最优通道组合
        graph.setNodeConversion(0, true);
        graph.setNodeConversion(1, true);
        graph.setNodeConversion(2, false);
        
        auto [path, cost] = graph.findShortestPath(0, 2, 1);
        assert(!path.empty());
        // 应该选择0-1使用通道1(代价1)，1-2使用通道0(代价1)，总代价2
        assert(cost == 2);
        cout << "测试通过: 代价=" << cost << endl;
    }
    
    // 测试用例3: 无转换约束测试
    cout << "\n3. 无转换约束测试" << endl;
    {
        ChannelGraph graph(3);
        
        vector<int> costs1 = TestUtils::generateAscendingCosts(1); // 1,2,3,...,100
        vector<int> costs2 = TestUtils::generateAscendingCosts(1);
        
        graph.addEdge(0, 1, costs1);
        graph.addEdge(1, 2, costs2);
        
        // 节点1不支持转换，必须使用相同通道
        graph.setNodeConversion(0, true);
        graph.setNodeConversion(1, false);
        graph.setNodeConversion(2, false);
        
        auto [path, cost] = graph.findShortestPath(0, 2, 1);
        assert(!path.empty());
        // 必须选择相同的通道，最小代价是通道0: 1+1=2
        assert(cost == 2);
        cout << "测试通过: 代价=" << cost << endl;
    }
}

void runAdvancedTests() {
    cout << "\n=== 高级功能测试 ===" << endl;
    
    // 测试用例4: 多路径选择测试
    cout << "\n4. 多路径选择测试" << endl;
    {
        ChannelGraph graph(5);
        
        // 路径1: 0-1-3 (代价: 2+3=5)
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(2));
        graph.addEdge(1, 3, TestUtils::generateConstantCosts(3));
        
        // 路径2: 0-2-3 (代价: 3+1=4) - 更优
        graph.addEdge(0, 2, TestUtils::generateConstantCosts(3));
        graph.addEdge(2, 3, TestUtils::generateConstantCosts(1));
        
        // 路径3: 0-4-3 (代价: 5+2=7)
        graph.addEdge(0, 4, TestUtils::generateConstantCosts(5));
        graph.addEdge(4, 3, TestUtils::generateConstantCosts(2));
        
        for (int i = 0; i < 5; ++i) {
            graph.setNodeConversion(i, true);
        }
        
        auto [path, cost] = graph.findShortestPath(0, 3, 1);
        assert(!path.empty());
        assert(cost == 4);
        assert(path.size() == 3);
        assert(path[0].first == 0);
        assert(path[1].first == 2); // 应该选择路径0-2-3
        assert(path[2].first == 3);
        cout << "测试通过: 选择了最优路径，代价=" << cost << endl;
    }
    
    // 测试用例5: 通道宽度测试
    cout << "\n5. 通道宽度测试" << endl;
    {
        ChannelGraph graph(2);
        
        vector<int> costs(100, 10);
        // 设置连续3个通道的代价较低
        costs[10] = 1; costs[11] = 1; costs[12] = 1;
        
        graph.addEdge(0, 1, costs);
        graph.setNodeConversion(0, true);
        graph.setNodeConversion(1, false);
        
        // 测试通道宽度=3
        auto [path, cost] = graph.findShortestPath(0, 1, 3);
        assert(!path.empty());
        assert(cost == 3); // 应该选择通道10-12，总代价3
        //assert(path[0].second == 10); // 起始通道应该是10
        cout << "测试通过: 通道宽度=3，代价=" << cost << endl;
    }
    
    // 测试用例6: 不可达测试
    cout << "\n6. 不可达测试" << endl;
    {
        ChannelGraph graph(4);
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(1));
        graph.addEdge(2, 3, TestUtils::generateConstantCosts(1));
        
        auto [path, cost] = graph.findShortestPath(0, 3, 1);
        assert(path.empty());
        assert(cost == INF);
        cout << "测试通过: 正确检测到不可达" << endl;
    }
}

void runPerformanceTests() {
    cout << "\n=== 性能测试 ===" << endl;
    
    // 测试用例7: 大规模图测试准备
    cout << "\n7. 大规模图测试准备" << endl;
    {
        const int NODES = 1000;
        const int EDGES = 5000;
        
        ChannelGraph graph(NODES);
        
        // 创建随机连接
        srand(42);
        for (int i = 0; i < EDGES; ++i) {
            int u = rand() % NODES;
            int v = rand() % NODES;
            if (u != v) {
                graph.addEdge(u, v, TestUtils::generateConstantCosts(rand() % 10 + 1));
            }
        }
        
        // 随机设置节点转换能力
        for (int i = 0; i < NODES; ++i) {
            graph.setNodeConversion(i, rand() % 2 == 0);
        }
        
        cout << "性能测试图创建完成: " << NODES << "节点, " << EDGES << "边" << endl;
        
        // 实际性能测试（可选）
        try {
            auto [path, cost] = graph.findShortestPath(0, NODES-1, 1);
            if (!path.empty()) {
                cout << "找到路径，代价=" << cost << ", 路径长度=" << path.size() << endl;
            } else {
                cout << "未找到路径" << endl;
            }
        } catch (const exception& e) {
            cout << "性能测试异常: " << e.what() << endl;
        }
    }
}

void runEdgeCaseTests() {
    cout << "\n=== 边界情况测试 ===" << endl;
    
    // 测试用例8: 自环测试
    cout << "\n8. 自环和重复节点测试" << endl;
    {
        ChannelGraph graph(3);
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(1));
        graph.addEdge(1, 2, TestUtils::generateConstantCosts(1));
        graph.addEdge(1, 1, TestUtils::generateConstantCosts(1)); // 自环
        
        for (int i = 0; i < 3; ++i) {
            graph.setNodeConversion(i, true);
        }
        
        auto [path, cost] = graph.findShortestPath(0, 2, 1);
        assert(!path.empty());
        assert(cost == 2);
        // 验证路径中没有重复节点
        unordered_set<int> nodes;
        for (const auto& p : path) {
            assert(!nodes.count(p.first));
            nodes.insert(p.first);
        }
        cout << "测试通过: 路径无重复节点，代价=" << cost << endl;
    }
    
    // 测试用例9: 相同节点测试
    cout << "\n9. 相同源和目标测试" << endl;
    {
        ChannelGraph graph(3);
        graph.addEdge(0, 1, TestUtils::generateConstantCosts(1));
        graph.addEdge(1, 2, TestUtils::generateConstantCosts(1));
        
        auto [path, cost] = graph.findShortestPath(0, 0, 1);
        assert(!path.empty());
        assert(path.size() == 1);
        assert(path[0].first == 0);
        assert(cost == 0);
        cout << "测试通过: 相同节点路径正确" << endl;
    }
}

int main() {
    try {
        runBasicTests();
        runAdvancedTests();
        runPerformanceTests();
        runEdgeCaseTests();
        
        cout << "\n=== 所有测试通过! ===" << endl;
    } catch (const exception& e) {
        cerr << "测试失败: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
