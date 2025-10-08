#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <functional>
#include <array>

using namespace std;

const int MAX_NODES = 10000;
const int CHANNELS = 100;
const int MAX_SEGMENTS = 3; // 1, 2, 3个连续通道

class OptimizedEfficientGraph {
private:
    int n;
    vector<bool> supports_switch;
    
    // 预计算所有边的段代价
    struct PrecomputedEdge {
        int to;
        array<int, CHANNELS> single_costs;      // 单通道代价
        array<int, CHANNELS-1> double_costs;    // 双连续通道最小代价
        array<int, CHANNELS-2> triple_costs;    // 三连续通道最小代价
        
        // 快速获取段代价
        int getSegmentCost(int start_channel, int segment_size) const {
            if (segment_size == 1) return single_costs[start_channel];
            if (segment_size == 2) return double_costs[start_channel];
            if (segment_size == 3) return triple_costs[start_channel];
            return INT_MAX;
        }
    };
    
    vector<vector<PrecomputedEdge>> adj;
    
    // 预计算边代价
    PrecomputedEdge precomputeEdge(int to, const array<int, CHANNELS>& costs) {
        PrecomputedEdge edge;
        edge.to = to;
        
        // 单通道代价
        copy(costs.begin(), costs.end(), edge.single_costs.begin());
        
        // 双连续通道代价
        for (int i = 0; i < CHANNELS - 1; i++) {
            edge.double_costs[i] = costs[i] + costs[i + 1];
        }
        
        // 三连续通道代价
        for (int i = 0; i < CHANNELS - 2; i++) {
            edge.triple_costs[i] = costs[i] + costs[i + 1] + costs[i + 2];
        }
        
        return edge;
    }

public:
    OptimizedEfficientGraph(int node_count) : n(node_count), adj(node_count), supports_switch(node_count, false) {}
    
    void setChannelSwitchSupport(int node_id, bool supports) {
        supports_switch[node_id] = supports;
    }
    
    void addEdge(int u, int v, const vector<int>& costs) {
        array<int, CHANNELS> cost_array;
        copy(costs.begin(), costs.end(), cost_array.begin());
        
        adj[u].push_back(precomputeEdge(v, cost_array));
        adj[v].push_back(precomputeEdge(u, cost_array));
    }
    
    int findMinCost(int source, int target) {
        const int STATE_COUNT = 101; // 100通道 + 特殊状态
        vector<int> dist(n * STATE_COUNT, INT_MAX);
        
        using State = pair<int, int>; // cost, state_id
        priority_queue<State, vector<State>, greater<State>> pq;
        
        int start_state = source * STATE_COUNT + 100;
        dist[start_state] = 0;
        pq.push({0, start_state});
        
        while (!pq.empty()) {
            auto [cost, state_id] = pq.top();
            pq.pop();
            
            if (cost > dist[state_id]) continue;
            
            int u = state_id / STATE_COUNT;
            int channel = state_id % STATE_COUNT;
            
            if (u == target && channel != 100) return cost;
            
            for (const PrecomputedEdge& edge : adj[u]) {
                int v = edge.to;
                
                if (channel == 100) {
                    // 开始新序列：尝试所有可能的段大小和起始通道
                    for (int seg_size = 1; seg_size <= 3; seg_size++) {
                        int max_start = CHANNELS - seg_size;
                        for (int start = 0; start <= max_start; start++) {
                            int segment_cost = edge.getSegmentCost(start, seg_size);
                            int new_channel = start + seg_size - 1;
                            int new_state = v * STATE_COUNT + new_channel;
                            int new_cost = cost + segment_cost;
                            
                            if (new_cost < dist[new_state]) {
                                dist[new_state] = new_cost;
                                pq.push({new_cost, new_state});
                            }
                        }
                    }
                } else {
                    // 继续当前序列
                    if (channel < CHANNELS - 1) {
                        int next_channel = channel + 1;
                        int channel_cost = edge.single_costs[next_channel];
                        int new_state = v * STATE_COUNT + next_channel;
                        int new_cost = cost + channel_cost;
                        
                        if (new_cost < dist[new_state]) {
                            dist[new_state] = new_cost;
                            pq.push({new_cost, new_state});
                        }
                    }
                    
                    // 重新开始序列（如果支持转换或必须重新开始）
                    if (supports_switch[u] || channel >= CHANNELS - 1) {
                        for (int seg_size = 1; seg_size <= 3; seg_size++) {
                            int max_start = CHANNELS - seg_size;
                            for (int start = 0; start <= max_start; start++) {
                                int segment_cost = edge.getSegmentCost(start, seg_size);
                                int new_channel = start + seg_size - 1;
                                int new_state = v * STATE_COUNT + new_channel;
                                int new_cost = cost + segment_cost;
                                
                                if (new_cost < dist[new_state]) {
                                    dist[new_state] = new_cost;
                                    pq.push({new_cost, new_state});
                                }
                            }
                        }
                    }
                }
            }
        }
        
        return -1;
    }
};
