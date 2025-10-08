#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <functional>
#include <array>
#include <unordered_map>
#include <algorithm>

using namespace std;

const int CHANNELS = 100;
const int MAX_SEGMENTS = 3;

class OptimizedEfficientGraph {
private:
    int n; // 节点数
    vector<bool> supports_switch; // 节点是否支持通道转换
    
    struct Edge {
        int to;
        array<int, CHANNELS> costs; // 100个通道的代价
        
        // 快速计算段代价
        int getSegmentCost(int start_channel, int segment_size) const {
            int total = 0;
            for (int i = 0; i < segment_size; i++) {
                total += costs[start_channel + i];
            }
            return total;
        }
    };
    
    vector<vector<Edge>> adj;
    
    // 路径记录结构
    struct PathState {
        int cost;
        int state;
        int prev_state;
        int start_channel; // 当前段的起始通道
        
        bool operator>(const PathState& other) const {
            return cost > other.cost;
        }
    };

public:
    OptimizedEfficientGraph(int node_count) : n(node_count), adj(node_count), supports_switch(node_count, false) {}
    
    void setChannelSwitchSupport(int node_id, bool supports) {
        if (node_id >= 0 && node_id < n) {
            supports_switch[node_id] = supports;
        }
    }
    
    void addEdge(int u, int v, const vector<int>& cost_vector) {
        if (cost_vector.size() != CHANNELS) {
            throw invalid_argument("Cost vector must have exactly 100 elements");
        }
        
        Edge edge_to_v, edge_to_u;
        edge_to_v.to = v;
        edge_to_u.to = u;
        
        for (int i = 0; i < CHANNELS; i++) {
            edge_to_v.costs[i] = cost_vector[i];
            edge_to_u.costs[i] = cost_vector[i];
        }
        
        adj[u].push_back(edge_to_v);
        adj[v].push_back(edge_to_u);
    }
    
    // 返回路径：vector<pair<节点ID, 起始通道ID>>，起始通道ID为-1表示未开始或结束
    vector<pair<int, int>> findMinCostPath(int source, int target) {
        const int STATE_COUNT = 101; // 100通道 + 特殊状态(100)
        const int TOTAL_STATES = n * STATE_COUNT;
        
        vector<int> dist(TOTAL_STATES, INT_MAX);
        vector<int> prev_state(TOTAL_STATES, -1);
        vector<int> start_channel(TOTAL_STATES, -1); // 记录段的起始通道
        
        priority_queue<PathState, vector<PathState>, greater<PathState>> pq;
        
        // 初始状态：源节点，未开始通道序列
        int start_state = source * STATE_COUNT + 100;
        dist[start_state] = 0;
        pq.push({0, start_state, -1, -1});
        
        int min_cost = INT_MAX;
        int best_final_state = -1;
        
        while (!pq.empty()) {
            PathState current = pq.top();
            pq.pop();
            
            if (current.cost > dist[current.state]) continue;
            if (current.cost > min_cost) continue;
            
            int u = current.state / STATE_COUNT;
            int channel = current.state % STATE_COUNT;
            
            // 记录前驱信息（第一次到达时记录）
            if (current.prev_state != -1 && prev_state[current.state] == -1) {
                prev_state[current.state] = current.prev_state;
                start_channel[current.state] = current.start_channel;
            }
            
            // 到达目标节点且已完成至少一个通道段
            if (u == target && channel != 100) {
                if (current.cost < min_cost) {
                    min_cost = current.cost;
                    best_final_state = current.state;
                }
                continue;
            }
            
            // 遍历所有邻接边
            for (const Edge& edge : adj[u]) {
                int v = edge.to;
                
                if (channel == 100) {
                    // 未开始状态：可以开始1、2、3连续通道段
                    for (int seg_size = 1; seg_size <= MAX_SEGMENTS; seg_size++) {
                        for (int start = 0; start <= CHANNELS - seg_size; start++) {
                            int segment_cost = edge.getSegmentCost(start, seg_size);
                            int new_cost = current.cost + segment_cost;
                            if (new_cost >= min_cost) continue;
                            
                            int new_channel = start + seg_size - 1;
                            int new_state = v * STATE_COUNT + new_channel;
                            
                            if (new_cost < dist[new_state]) {
                                dist[new_state] = new_cost;
                                pq.push({new_cost, new_state, current.state, start});
                            }
                        }
                    }
                } else {
                    // 已有当前通道状态
                    int current_channel = channel;
                    
                    // 情况1：继续当前通道序列（如果可能）
                    if (current_channel < CHANNELS - 1) {
                        int next_channel = current_channel + 1;
                        int channel_cost = edge.costs[next_channel];
                        int new_cost = current.cost + channel_cost;
                        
                        if (new_cost < min_cost) {
                            int new_state = v * STATE_COUNT + next_channel;
                            
                            if (new_cost < dist[new_state]) {
                                dist[new_state] = new_cost;
                                // 继续序列时，起始通道保持不变（使用前一个状态的起始通道）
                                int continued_start_channel = current.start_channel;
                                if (continued_start_channel == -1) {
                                    // 如果之前没有记录起始通道，说明是继续序列的开始
                                    continued_start_channel = current_channel;
                                }
                                pq.push({new_cost, new_state, current.state, continued_start_channel});
                            }
                        }
                    }
                    
                    // 情况2：重新开始新的通道序列
                    bool can_restart = supports_switch[u] || current_channel >= CHANNELS - 1;
                    if (can_restart) {
                        for (int seg_size = 1; seg_size <= MAX_SEGMENTS; seg_size++) {
                            for (int start = 0; start <= CHANNELS - seg_size; start++) {
                                int segment_cost = edge.getSegmentCost(start, seg_size);
                                int new_cost = current.cost + segment_cost;
                                if (new_cost >= min_cost) continue;
                                
                                int new_channel = start + seg_size - 1;
                                int new_state = v * STATE_COUNT + new_channel;
                                
                                if (new_cost < dist[new_state]) {
                                    dist[new_state] = new_cost;
                                    pq.push({new_cost, new_state, current.state, start});
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // 重构路径
        return reconstructPath(best_final_state, prev_state, start_channel, source, target, STATE_COUNT);
    }
    
private:
    vector<pair<int, int>> reconstructPath(int final_state, 
                                         const vector<int>& prev_state,
                                         const vector<int>& start_channel,
                                         int source, int target,
                                         int STATE_COUNT) {
        vector<pair<int, int>> path;
        
        if (final_state == -1) {
            return path; // 空路径，表示不可达
        }
        
        // 反向追踪路径
        vector<pair<int, int>> reverse_path;
        int current_state = final_state;
        
        while (current_state != -1) {
            int node = current_state / STATE_COUNT;
            int channel_start = start_channel[current_state];
            reverse_path.push_back({node, channel_start});
            current_state = prev_state[current_state];
        }
        
        // 反转路径并处理起始通道信息
        for (int i = reverse_path.size() - 1; i >= 0; i--) {
            int node = reverse_path[i].first;
            int channel = reverse_path[i].second;
            
            // 源节点和目标节点的起始通道设为-1
            if (node == source || node == target) {
                path.push_back({node, -1});
            } else {
                path.push_back({node, channel});
            }
        }
        
        return path;
    }
};

// 测试用例生成器
class TestCaseGenerator {
public:
    static vector<int> generateConstantCosts(int value = 1) {
        vector<int> costs(CHANNELS, value);
        return costs;
    }
    
    static vector<int> generateLinearCosts(int base = 1, int step = 1) {
        vector<int> costs(CHANNELS);
        for (int i = 0; i < CHANNELS; i++) {
            costs[i] = base + i * step;
        }
        return costs;
    }
    
    static vector<int> generateRandomCosts(int min_cost = 1, int max_cost = 100) {
        vector<int> costs(CHANNELS);
        srand(time(nullptr));
        for (int i = 0; i < CHANNELS; i++) {
            costs[i] = rand() % (max_cost - min_cost + 1) + min_cost;
        }
        return costs;
    }
    
    static vector<int> generateLowMiddleHighCosts() {
        vector<int> costs(CHANNELS);
        for (int i = 0; i < CHANNELS; i++) {
            if (i < 33) costs[i] = 1;  // 低成本段
            else if (i < 66) costs[i] = 10; // 中等成本段
            else costs[i] = 100; // 高成本段
        }
        return costs;
    }
};

// 测试函数
void runTestCases() {
    cout << "=== 测试用例开始 ===" << endl;
    
    // 测试用例1：简单线性图
    {
        cout << "\n测试用例1: 简单线性图" << endl;
        OptimizedEfficientGraph graph(3);
        
        // 所有节点支持通道转换
        graph.setChannelSwitchSupport(0, true);
        graph.setChannelSwitchSupport(1, true);
        graph.setChannelSwitchSupport(2, true);
        
        // 添加边，使用线性递增代价
        graph.addEdge(0, 1, TestCaseGenerator::generateLinearCosts(1, 1));
        graph.addEdge(1, 2, TestCaseGenerator::generateLinearCosts(1, 1));
        
        auto path = graph.findMinCostPath(0, 2);
        
        if (path.empty()) {
            cout << "无法到达目标节点" << endl;
        } else {
            cout << "路径: ";
            for (const auto& p : path) {
                cout << "(" << p.first << ", " << p.second << ") ";
            }
            cout << endl;
        }
    }
    
    // 测试用例2：包含不支持转换的节点
    {
        cout << "\n测试用例2: 包含不支持转换的节点" << endl;
        OptimizedEfficientGraph graph(4);
        
        graph.setChannelSwitchSupport(0, true);
        graph.setChannelSwitchSupport(1, false); // 关键节点不支持转换
        graph.setChannelSwitchSupport(2, true);
        graph.setChannelSwitchSupport(3, true);
        
        // 使用不同的代价模式
        graph.addEdge(0, 1, TestCaseGenerator::generateLowMiddleHighCosts());
        graph.addEdge(1, 2, TestCaseGenerator::generateLinearCosts(10, 2));
        graph.addEdge(2, 3, TestCaseGenerator::generateRandomCosts(1, 5));
        
        auto path = graph.findMinCostPath(0, 3);
        
        if (path.empty()) {
            cout << "无法到达目标节点" << endl;
        } else {
            cout << "路径: ";
            for (const auto& p : path) {
                cout << "(" << p.first << ", " << p.second << ") ";
            }
            cout << endl;
        }
    }
    
    // 测试用例3：复杂网络
    {
        cout << "\n测试用例3: 复杂网络" << endl;
        OptimizedEfficientGraph graph(6);
        
        // 设置部分节点支持转换
        graph.setChannelSwitchSupport(0, true);
        graph.setChannelSwitchSupport(1, false);
        graph.setChannelSwitchSupport(2, true);
        graph.setChannelSwitchSupport(3, false);
        graph.setChannelSwitchSupport(4, true);
        graph.setChannelSwitchSupport(5, true);
        
        // 创建网格状连接
        graph.addEdge(0, 1, TestCaseGenerator::generateLinearCosts(1, 1));
        graph.addEdge(0, 2, TestCaseGenerator::generateLinearCosts(5, 1));
        graph.addEdge(1, 3, TestCaseGenerator::generateLinearCosts(2, 2));
        graph.addEdge(2, 3, TestCaseGenerator::generateLinearCosts(1, 3));
        graph.addEdge(2, 4, TestCaseGenerator::generateRandomCosts(1, 10));
        graph.addEdge(3, 5, TestCaseGenerator::generateConstantCosts(8));
        graph.addEdge(4, 5, TestCaseGenerator::generateLinearCosts(3, 1));
        
        auto path = graph.findMinCostPath(0, 5);
        
        if (path.empty()) {
            cout << "无法到达目标节点" << endl;
        } else {
            cout << "路径: ";
            for (const auto& p : path) {
                cout << "(" << p.first << ", " << p.second << ") ";
            }
            cout << endl;
        }
    }
    
    // 测试用例4：性能测试（中等规模）
    {
        cout << "\n测试用例4: 中等规模性能测试" << endl;
        const int NODES = 100;
        OptimizedEfficientGraph graph(NODES);
        
        // 随机设置节点转换支持
        srand(time(nullptr));
        for (int i = 0; i < NODES; i++) {
            graph.setChannelSwitchSupport(i, rand() % 2 == 0);
        }
        
        // 创建随机连接
        for (int i = 0; i < NODES - 1; i++) {
            graph.addEdge(i, i + 1, TestCaseGenerator::generateRandomCosts(1, 20));
        }
        // 添加一些额外连接
        for (int i = 0; i < NODES / 2; i++) {
            int u = rand() % NODES;
            int v = rand() % NODES;
            if (u != v) {
                graph.addEdge(u, v, TestCaseGenerator::generateRandomCosts(1, 30));
            }
        }
        
        auto path = graph.findMinCostPath(0, NODES - 1);
        
        if (path.empty()) {
            cout << "无法到达目标节点" << endl;
        } else {
            cout << "找到路径，节点数: " << path.size() << endl;
            // 显示部分路径信息
            cout << "前3个节点: ";
            for (int i = 0; i < min(3, (int)path.size()); i++) {
                cout << "(" << path[i].first << ", " << path[i].second << ") ";
            }
            if (path.size() > 3) {
                cout << "... 最后节点: (" << path.back().first << ", " << path.back().second << ")";
            }
            cout << endl;
        }
    }
    
    // 测试用例5：边界情况 - 单节点
    {
        cout << "\n测试用例5: 单节点边界情况" << endl;
        OptimizedEfficientGraph graph(1);
        graph.setChannelSwitchSupport(0, true);
        
        auto path = graph.findMinCostPath(0, 0);
        
        if (path.empty()) {
            cout << "错误：单节点应该能找到路径" << endl;
        } else {
            cout << "路径: ";
            for (const auto& p : path) {
                cout << "(" << p.first << ", " << p.second << ") ";
            }
            cout << endl;
        }
    }
    
    cout << "\n=== 测试用例结束 ===" << endl;
}

int main() {
    try {
        runTestCases();
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
    }
    
    return 0;
}
