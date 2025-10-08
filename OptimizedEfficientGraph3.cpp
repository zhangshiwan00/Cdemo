#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <functional>
#include <array>
#include <unordered_set>
#include <algorithm>
#include <memory>

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
    
    // 状态定义
    struct State {
        int cost;
        int node;
        int channel;        // 当前通道 (-1表示未开始)
        int consecutive;    // 连续通道计数
        unordered_set<int> visited; // 已访问节点
        
        bool operator>(const State& other) const {
            return cost > other.cost;
        }
        
        // 复制构造函数
        State(const State& other) 
            : cost(other.cost), node(other.node), channel(other.channel), 
              consecutive(other.consecutive), visited(other.visited) {}
        
        State(int c, int n, int ch, int cons) 
            : cost(c), node(n), channel(ch), consecutive(cons) {}
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
        if (source == target) {
            return {{source, -1}}; // 特殊情况：源节点就是目标节点
        }
        
        // 使用优先队列进行Dijkstra搜索
        priority_queue<State, vector<State>, greater<State>> pq;
        
        // 初始状态
        State start(0, source, -1, 0);
        start.visited.insert(source);
        pq.push(start);
        
        // 记录最佳路径
        int min_cost = INT_MAX;
        shared_ptr<State> best_state = nullptr;
        unordered_map<int, shared_ptr<State>> best_predecessor; // 用于重建路径
        
        while (!pq.empty()) {
            State current = pq.top();
            pq.pop();
            
            // 剪枝：如果当前代价已经大于已知最小代价
            if (current.cost > min_cost) continue;
            
            int u = current.node;
            
            // 到达目标节点
            if (u == target) {
                if (current.cost < min_cost) {
                    min_cost = current.cost;
                    best_state = make_shared<State>(current);
                }
                continue;
            }
            
            // 遍历所有邻接边
            for (const Edge& edge : adj[u]) {
                int v = edge.to;
                
                // 检查节点是否已访问
                if (current.visited.find(v) != current.visited.end()) {
                    continue; // 节点已访问，跳过
                }
                
                if (current.channel == -1) {
                    // 未开始状态：可以开始1、2、3连续通道段
                    for (int seg_size = 1; seg_size <= MAX_SEGMENTS; seg_size++) {
                        for (int start = 0; start <= CHANNELS - seg_size; start++) {
                            int segment_cost = edge.getSegmentCost(start, seg_size);
                            int new_cost = current.cost + segment_cost;
                            if (new_cost >= min_cost) continue;
                            
                            State new_state(new_cost, v, start + seg_size - 1, seg_size);
                            new_state.visited = current.visited;
                            new_state.visited.insert(v);
                            
                            // 记录前驱信息用于重建路径
                            int state_key = v * 1000 + (start + seg_size - 1) * 10 + seg_size;
                            best_predecessor[state_key] = make_shared<State>(current);
                            
                            pq.push(new_state);
                        }
                    }
                } else {
                    // 已有当前通道状态
                    int current_channel = current.channel;
                    
                    // 情况1：继续当前通道序列（如果可能）
                    if (current_channel < CHANNELS - 1 && current.consecutive < MAX_SEGMENTS) {
                        int next_channel = current_channel + 1;
                        int channel_cost = edge.costs[next_channel];
                        int new_cost = current.cost + channel_cost;
                        
                        if (new_cost < min_cost) {
                            State new_state(new_cost, v, next_channel, current.consecutive + 1);
                            new_state.visited = current.visited;
                            new_state.visited.insert(v);
                            
                            int state_key = v * 1000 + next_channel * 10 + (current.consecutive + 1);
                            best_predecessor[state_key] = make_shared<State>(current);
                            
                            pq.push(new_state);
                        }
                    }
                    
                    // 情况2：重新开始新的通道序列
                    bool can_restart = supports_switch[u] || 
                                     current_channel >= CHANNELS - 1 || 
                                     current.consecutive == MAX_SEGMENTS;
                    
                    if (can_restart) {
                        for (int seg_size = 1; seg_size <= MAX_SEGMENTS; seg_size++) {
                            for (int start = 0; start <= CHANNELS - seg_size; start++) {
                                int segment_cost = edge.getSegmentCost(start, seg_size);
                                int new_cost = current.cost + segment_cost;
                                if (new_cost >= min_cost) continue;
                                
                                State new_state(new_cost, v, start + seg_size - 1, seg_size);
                                new_state.visited = current.visited;
                                new_state.visited.insert(v);
                                
                                int state_key = v * 1000 + (start + seg_size - 1) * 10 + seg_size;
                                best_predecessor[state_key] = make_shared<State>(current);
                                
                                pq.push(new_state);
                            }
                        }
                    }
                }
            }
        }
        
        // 重建路径
        return reconstructPath(best_state, best_predecessor, source, target);
    }
    
private:
    vector<pair<int, int>> reconstructPath(shared_ptr<State> final_state, 
                                         const unordered_map<int, shared_ptr<State>>& predecessor,
                                         int source, int target) {
        vector<pair<int, int>> path;
        
        if (!final_state) {
            return path; // 空路径，表示不可达
        }
        
        // 反向追踪路径
        vector<shared_ptr<State>> reverse_states;
        shared_ptr<State> current = final_state;
        
        while (current) {
            reverse_states.push_back(current);
            
            if (current->node == source) {
                break;
            }
            
            // 查找前驱状态
            int state_key = current->node * 1000 + current->channel * 10 + current->consecutive;
            auto it = predecessor.find(state_key);
            if (it == predecessor.end()) {
                break; // 无法继续追踪
            }
            
            current = it->second;
        }
        
        // 反转路径
        for (int i = reverse_states.size() - 1; i >= 0; i--) {
            const auto& state = reverse_states[i];
            
            // 确定起始通道
            int start_channel = -1;
            if (state->channel != -1) {
                start_channel = state->channel - state->consecutive + 1;
            }
            
            // 源节点和目标节点的起始通道设为-1
            if (state->node == source || state->node == target) {
                path.push_back({state->node, -1});
            } else {
                path.push_back({state->node, start_channel});
            }
        }
        
        return path;
    }
};

// 测试用例生成器
class TestCaseGenerator {
public:
    static vector<int> generateConstantCosts(int value = 1) {
        return vector<int>(CHANNELS, value);
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
            if (i < 33) costs[i] = 1;
            else if (i < 66) costs[i] = 10;
            else costs[i] = 100;
        }
        return costs;
    }
};

// 路径验证函数
bool validatePath(const vector<pair<int, int>>& path, int source, int target) {
    if (path.empty()) {
        cout << "路径为空" << endl;
        return false;
    }
    
    // 检查源节点和目标节点
    if (path.front().first != source) {
        cout << "路径起始节点错误" << endl;
        return false;
    }
    if (path.back().first != target) {
        cout << "路径目标节点错误" << endl;
        return false;
    }
    
    // 检查节点重复
    unordered_set<int> visited_nodes;
    for (const auto& p : path) {
        if (visited_nodes.find(p.first) != visited_nodes.end()) {
            cout << "节点重复: " << p.first << endl;
            return false;
        }
        visited_nodes.insert(p.first);
    }
    
    cout << "路径验证通过，节点数: " << path.size() << endl;
    return true;
}

// 测试函数
void runTestCases() {
    cout << "=== 测试用例开始 ===" << endl;
    
    // 测试用例1：简单线性图
    {
        cout << "\n测试用例1: 简单线性图" << endl;
        OptimizedEfficientGraph graph(3);
        
        graph.setChannelSwitchSupport(0, true);
        graph.setChannelSwitchSupport(1, true);
        graph.setChannelSwitchSupport(2, true);
        
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
            validatePath(path, 0, 2);
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
            validatePath(path, 0, 3);
        }
    }
    
    // 测试用例3：复杂网络，确保无重复节点
    {
        cout << "\n测试用例3: 复杂网络" << endl;
        OptimizedEfficientGraph graph(6);
        
        graph.setChannelSwitchSupport(0, true);
        graph.setChannelSwitchSupport(1, false);
        graph.setChannelSwitchSupport(2, true);
        graph.setChannelSwitchSupport(3, false);
        graph.setChannelSwitchSupport(4, true);
        graph.setChannelSwitchSupport(5, true);
        
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
            validatePath(path, 0, 5);
        }
    }
    
    // 测试用例4：环形图测试（确保不会进入死循环）
    {
        cout << "\n测试用例4: 环形图" << endl;
        OptimizedEfficientGraph graph(4);
        
        graph.setChannelSwitchSupport(0, true);
        graph.setChannelSwitchSupport(1, true);
        graph.setChannelSwitchSupport(2, true);
        graph.setChannelSwitchSupport(3, true);
        
        // 创建环形结构
        graph.addEdge(0, 1, TestCaseGenerator::generateConstantCosts(1));
        graph.addEdge(1, 2, TestCaseGenerator::generateConstantCosts(1));
        graph.addEdge(2, 3, TestCaseGenerator::generateConstantCosts(1));
        graph.addEdge(3, 0, TestCaseGenerator::generateConstantCosts(1)); // 形成环
        graph.addEdge(1, 3, TestCaseGenerator::generateConstantCosts(5)); // 捷径
        
        auto path = graph.findMinCostPath(0, 3);
        
        if (path.empty()) {
            cout << "无法到达目标节点" << endl;
        } else {
            cout << "路径: ";
            for (const auto& p : path) {
                cout << "(" << p.first << ", " << p.second << ") ";
            }
            cout << endl;
            validatePath(path, 0, 3);
            
            // 验证路径确实没有重复节点
            if (path.size() > 0) {
                cout << "路径长度: " << path.size() << endl;
            }
        }
    }
    
    // 测试用例5：性能测试（中等规模）
    {
        cout << "\n测试用例5: 中等规模性能测试" << endl;
        const int NODES = 50; // 减小规模以便快速测试
        OptimizedEfficientGraph graph(NODES);
        
        srand(time(nullptr));
        for (int i = 0; i < NODES; i++) {
            graph.setChannelSwitchSupport(i, rand() % 2 == 0);
        }
        
        // 创建随机连接
        for (int i = 0; i < NODES - 1; i++) {
            graph.addEdge(i, i + 1, TestCaseGenerator::generateRandomCosts(1, 20));
        }
        
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
            validatePath(path, 0, NODES - 1);
        }
    }
    
    // 测试用例6：边界情况 - 单节点
    {
        cout << "\n测试用例6: 单节点边界情况" << endl;
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
            validatePath(path, 0, 0);
        }
    }
    
    // 测试用例7：不可达情况
    {
        cout << "\n测试用例7: 不可达情况" << endl;
        OptimizedEfficientGraph graph(3);
        
        graph.setChannelSwitchSupport(0, true);
        graph.setChannelSwitchSupport(1, true);
        graph.setChannelSwitchSupport(2, true);
        
        // 只连接0和1，不连接2
        graph.addEdge(0, 1, TestCaseGenerator::generateConstantCosts(1));
        
        auto path = graph.findMinCostPath(0, 2);
        
        if (path.empty()) {
            cout << "正确：节点不可达" << endl;
        } else {
            cout << "错误：应该找不到路径" << endl;
            validatePath(path, 0, 2);
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
