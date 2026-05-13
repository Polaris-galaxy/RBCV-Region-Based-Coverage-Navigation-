/**
 * 与 ``exploration.planner.plan_exploration_tour`` 对齐的 NN + 2-opt（格子图为无向加权）。
 *
 * 标准输入（ASCII）：
 *   第一行：n start_idx num_edges
 *   随后 num_edges 行：u v w   （有向边；通常成对出现 u->v 与 v->u）
 * 标准输出：一行，空格分隔的节点下标，闭合回路（首点重复一次在末尾），与 Python 一致。
 */
#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

static double edge_w(const std::vector<std::unordered_map<int, double>>& adj, int a, int b) {
  auto it = adj[static_cast<size_t>(a)].find(b);
  if (it == adj[static_cast<size_t>(a)].end()) {
    return 1e6;
  }
  return it->second;
}

static double tour_len_open(const std::vector<int>& seq,
                          const std::vector<std::unordered_map<int, double>>& adj) {
  if (seq.size() <= 1) {
    return 0.0;
  }
  double s = 0.0;
  for (size_t i = 0; i + 1 < seq.size(); ++i) {
    s += edge_w(adj, seq[i], seq[i + 1]);
  }
  s += edge_w(adj, seq.back(), seq.front());
  return s;
}

static std::vector<int> two_opt(std::vector<int> seq,
                                const std::vector<std::unordered_map<int, double>>& adj) {
  bool improved = true;
  std::vector<int> best_seq = seq;
  double best = tour_len_open(seq, adj);
  const double eps = 1e-9;
  while (improved) {
    improved = false;
    for (int i = 1; i < static_cast<int>(seq.size()) - 2; ++i) {
      for (int k = i + 1; k < static_cast<int>(seq.size()); ++k) {
        if (k - i == 1) {
          continue;
        }
        std::vector<int> neu;
        neu.reserve(seq.size());
        neu.insert(neu.end(), seq.begin(), seq.begin() + i);
        neu.insert(neu.end(), seq.begin() + i, seq.begin() + k);
        std::reverse(neu.begin() + i, neu.end());
        neu.insert(neu.end(), seq.begin() + k, seq.end());
        double ln = tour_len_open(neu, adj);
        if (ln + eps < best) {
          best = ln;
          best_seq = neu;
          seq = neu;
          improved = true;
          break;
        }
      }
      if (improved) {
        break;
      }
    }
  }
  return best_seq;
}

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);

  int n = 0;
  int start = 0;
  int num_edges = 0;
  if (!(std::cin >> n >> start >> num_edges)) {
    std::cerr << "rbcv_plan_tour: 需要首行: n start_idx num_edges\n";
    return 2;
  }
  if (n <= 0) {
    std::cout << "\n";
    return 0;
  }
  std::vector<std::unordered_map<int, double>> adj(static_cast<size_t>(n));
  for (int e = 0; e < num_edges; ++e) {
    int u = 0;
    int v = 0;
    double w = 0.0;
    if (!(std::cin >> u >> v >> w)) {
      std::cerr << "rbcv_plan_tour: 边数据不足\n";
      return 2;
    }
    if (u < 0 || u >= n || v < 0 || v >= n) {
      continue;
    }
    adj[static_cast<size_t>(u)][v] = w;
  }

  if (start < 0 || start >= n) {
    start = 0;
  }

  std::vector<char> seen(static_cast<size_t>(n), 0);
  std::vector<int> order;
  order.reserve(static_cast<size_t>(n));
  order.push_back(start);
  seen[static_cast<size_t>(start)] = 1;
  int cur = start;

  while (static_cast<int>(order.size()) < n) {
    int best = -1;
    double best_w = std::numeric_limits<double>::infinity();
    for (const auto& [nb, wt] : adj[static_cast<size_t>(cur)]) {
      if (seen[static_cast<size_t>(nb)]) {
        continue;
      }
      if (wt < best_w) {
        best_w = wt;
        best = nb;
      }
    }
    if (best < 0) {
      break;
    }
    order.push_back(best);
    seen[static_cast<size_t>(best)] = 1;
    cur = best;
  }

  auto cyc = two_opt(std::move(order), adj);
  for (size_t i = 0; i < cyc.size(); ++i) {
    if (i) {
      std::cout << ' ';
    }
    std::cout << cyc[i];
  }
  if (!cyc.empty()) {
    std::cout << ' ' << cyc.front();
  }
  std::cout << '\n';
  return 0;
}
