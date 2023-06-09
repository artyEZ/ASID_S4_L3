#include <vector>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <limits>
#include <random>

template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance distance;
    };

    bool has_vertex(const Vertex& v) const {
        return adjacency_list.count(v) > 0;
    }

    void add_vertex(const Vertex& v) {
        adjacency_list[v] = std::vector<Edge>();
    }

    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v))
            return false;

        adjacency_list.erase(v);

        // Удалить все ребра, ведущие в вершину v
        for (auto& pair : adjacency_list) {
            auto& edges = pair.second;
            edges.erase(std::remove_if(edges.begin(), edges.end(),
                [&v](const Edge& e) { return e.to == v; }),
                edges.end());
        }

        return true;
    }

    std::vector<Vertex> vertices() const {
        std::vector<Vertex> result;
        for (const auto& pair : adjacency_list) {
            result.push_back(pair.first);
        }
        return result;
    }

    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        if (!has_vertex(from) || !has_vertex(to))
            return;

        adjacency_list[from].push_back(Edge{ from, to, d });
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        if (!has_vertex(from) || !has_vertex(to))
            return false;

        auto& edges = adjacency_list[from];
        auto it = std::find_if(edges.begin(), edges.end(),
            [&to](const Edge& e) { return e.to == to; });

        if (it != edges.end()) {
            edges.erase(it);
            return true;
        }

        return false;
    }

    bool remove_edge(const Edge& e) {
        return remove_edge(e.from, e.to);
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from) || !has_vertex(to))
            return false;

        const auto& edges = adjacency_list.at(from);
        return std::any_of(edges.begin(), edges.end(),
            [&to](const Edge& e) { return e.to == to; });
    }

    bool has_edge(const Edge& e) {
        return has_edge(e.from, e.to) && e.distance == get_edge_distance(e.from, e.to);
    }

    std::vector<Edge> edges(const Vertex& vertex) {
        if (!has_vertex(vertex))
            return {};

        return adjacency_list[vertex];
    }

    size_t order() const {
        return adjacency_list.size();
    }

    size_t degree() const {
        size_t max_degree = 0;
        for (const auto& pair : adjacency_list) {
            max_degree = std::max(max_degree, pair.second.size());
        }
        return max_degree;
    }

    std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from) || !has_vertex(to))
            return {};

        std::unordered_map<Vertex, Distance> distances;
        std::unordered_map<Vertex, Vertex> previous;
        std::priority_queue<std::pair<Distance, Vertex>,
            std::vector<std::pair<Distance, Vertex>>,
            std::greater<std::pair<Distance, Vertex>>> pq;

        for (const auto& pair : adjacency_list) {
            distances[pair.first] = std::numeric_limits<Distance>::max();
        }

        distances[from] = 0;
        pq.push(std::make_pair(0, from));

        while (!pq.empty()) {
            Vertex u = pq.top().second;
            pq.pop();

            if (u == to)
                break;

            for (const auto& edge : adjacency_list.at(u)) {
                Distance alt = distances[u] + edge.distance;
                if (alt < distances[edge.to]) {
                    distances[edge.to] = alt;
                    previous[edge.to] = u;
                    pq.push(std::make_pair(alt, edge.to));
                }
            }
        }

        std::vector<Edge> path;
        Vertex current = to;
        while (previous.count(current) > 0) {
            Vertex prev = previous[current];
            Distance distance = get_edge_distance(prev, current);
            path.push_back(Edge{ prev, current, distance });
            current = prev;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<Vertex> walk(const Vertex& start_vertex) const {
        if (!has_vertex(start_vertex))
            return {};

        std::vector<Vertex> visited;
        std::queue<Vertex> queue;
        std::unordered_map<Vertex, bool> visited_map;

        queue.push(start_vertex);
        visited_map[start_vertex] = true;

        while (!queue.empty()) {
            Vertex current = queue.front();
            queue.pop();

            visited.push_back(current);

            const auto& edges = adjacency_list.at(current);
            for (const auto& edge : edges) {
                if (!visited_map[edge.to]) {
                    queue.push(edge.to);
                    visited_map[edge.to] = true;
                }
            }
        }

        return visited;
    }


    Vertex find_farthest_from_neighbors(){
        Vertex farthest_vertex;
        double max_average_distance = 0;

        std::vector<Vertex> vertices = this->vertices();

        for (const auto& vertex : vertices) {
            std::vector<Edge> edges = this->edges(vertex);

            if (edges.size() < 2)
                continue;

            double total_distance = 0;

            for (const auto& edge : edges) {
                total_distance += edge.distance;
            }

            double average_distance = total_distance / edges.size();

            if (average_distance > max_average_distance) {
                max_average_distance = average_distance;
                farthest_vertex = vertex;
            }
        }

        return farthest_vertex;
    }

private:
    std::unordered_map<Vertex, std::vector<Edge>> adjacency_list;

    Distance get_edge_distance(const Vertex& from, const Vertex& to) const {
        const auto& edges = adjacency_list.at(from);
        auto it = std::find_if(edges.begin(), edges.end(),
            [&to](const Edge& e) { return e.to == to; });
        return (it != edges.end()) ? it->distance : std::numeric_limits<Distance>::max();
    }
};

int main() {
    // Создание графа
    Graph<int, double> graph;

    // Добавление вершин
    for (int i = 1; i <= 10; i++) {
        graph.add_vertex(i);
    }

    // Добавление ребер
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(1, 10);

    for (int i = 1; i <= 10; i++) {
        for (int j = 1; j <= 10; j++) {
            if (i != j) {
                double distance = dist(gen) / 10.0;
                graph.add_edge(i, j, distance);
            }
        }
    }

    // Проверка наличия вершин и ребер
    std::cout << "Has vertex 5: " << graph.has_vertex(5) << std::endl;
    std::cout << "Has edge (2, 3): " << graph.has_edge(2, 3) << std::endl;

    // Удаление вершины и ребра
    graph.remove_vertex(3);
    graph.remove_edge(4, 7);

    // Поиск кратчайшего пути
    std::vector<Graph<int, double>::Edge> shortestPath = graph.shortest_path(1, 10);
    std::cout << "Shortest path from 1 to 10: ";
    for (const auto& edge : shortestPath) {
        std::cout << "(" << edge.from << ", " << edge.to << ") ";
    }
    std::cout << std::endl;

    // Обход графа в ширину
    std::vector<int> walk = graph.walk(1);
    std::cout << "Walk starting from vertex 1: ";
    for (const auto& vertex : walk) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;

    // Поиск травмпункта, наиболее удаленного от своих прямых соседей
    int farthest_vertex = graph.find_farthest_from_neighbors();
    std::cout << "Farthest vertex from neighbors: " << farthest_vertex << std::endl;

    return 0;
}