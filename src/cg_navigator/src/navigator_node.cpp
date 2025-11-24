#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <vector>
#include <queue>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

int id(int r, int c, int W) {
    return r * W + c;
}

// Transforma vetor flat do labirinto em matrix
std::vector<std::vector<char>> flat_to_matrix(
    const std::vector<std::string> &flat, int W, int H)
{
    std::vector<std::vector<char>> grid(H, std::vector<char>(W));

    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++)
            grid[r][c] = flat[r * W + c][0];

    return grid;
}

// Transforma matrix do labirinto em grafo
std::vector<std::vector<int>> build_graph(
    const std::vector<std::vector<char>> &grid,
    int H, int W, int &start, int &goal)
{
    std::vector<std::vector<int>> graph(H * W);

    auto free_cell = [&](char v){
        return (v == 'f' || v == 'r' || v == 't');
    };

    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) {

            char v = grid[r][c];

            if (v == 'r') start = id(r, c, W);
            if (v == 't') goal  = id(r, c, W);

            if (!free_cell(v)) continue;

            int u = id(r, c, W);

            if (r > 0 && free_cell(grid[r-1][c]))
                graph[u].push_back(id(r-1, c, W));
            if (r+1 < H && free_cell(grid[r+1][c]))
                graph[u].push_back(id(r+1, c, W));
            if (c > 0 && free_cell(grid[r][c-1]))
                graph[u].push_back(id(r, c-1, W));
            if (c+1 < W && free_cell(grid[r][c+1]))
                graph[u].push_back(id(r, c+1, W));
        }
    }

    return graph;
}

// Algoritmo de breath firts search
std::vector<int> bfs_path(
    const std::vector<std::vector<int>>& graph,
    int start,
    int goal)
{
    std::queue<int> q;
    std::vector<bool> visited(graph.size(), false);
    std::vector<int> parent(graph.size(), -1);

    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
        int u = q.front(); q.pop();

        if (u == goal) break;

        for (int v : graph[u]) {
            if (!visited[v]) {
                visited[v] = true;
                parent[v] = u;
                q.push(v);
            }
        }
    }

    if (!visited[goal]) return {};

    std::vector<int> path;
    for (int cur = goal; cur != -1; cur = parent[cur])
        path.push_back(cur);

    std::reverse(path.begin(), path.end());
    return path;
}

// Converte solução do do caminho para movimentos do robo
std::vector<std::string> path_to_moves(
    const std::vector<int>& path, int W)
{
    std::vector<std::string> moves;

    for (int i = 0; i < (int)path.size()-1; i++) {
        int a = path[i];
        int b = path[i+1];

        int ra = a / W, ca = a % W;
        int rb = b / W, cb = b % W;

        if (rb == ra - 1) moves.push_back("up");
        else if (rb == ra + 1) moves.push_back("down");
        else if (cb == ca - 1) moves.push_back("left");
        else if (cb == ca + 1) moves.push_back("right");
    }

    return moves;
}

class Navigator : public rclcpp::Node {
public:
    Navigator()
    : Node("navigator_node")
    {
        RCLCPP_INFO(this->get_logger(), "Navigator iniciado!");

        get_map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        move_client_    = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        timer_ = this->create_wall_timer(
            1s, std::bind(&Navigator::call_get_map, this)
        );
    }

private:

    void call_get_map()
    {
        timer_->cancel();

        if (!get_map_client_->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "Aguardando /get_map...");
            timer_->reset();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Chamando /get_map...");

        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();

        get_map_client_->async_send_request(
            req,
            std::bind(&Navigator::on_map_received, this, std::placeholders::_1)
        );
    }

    void on_map_received(
        rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future)
    {
        auto resp = future.get();

        int W = resp->occupancy_grid_shape[0];
        int H = resp->occupancy_grid_shape[1];

        auto grid = flat_to_matrix(resp->occupancy_grid_flattened, W, H);

        int start = -1, goal = -1;
        auto graph = build_graph(grid, H, W, start, goal);

        if (start == -1 || goal == -1) {
            RCLCPP_ERROR(this->get_logger(), "Start ou Goal não encontrados!");
            return;
        }

        auto path = bfs_path(graph, start, goal);
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "SEM CAMINHO!");
            return;
        }

        auto moves = path_to_moves(path, W);
        RCLCPP_INFO(this->get_logger(), "Executando %zu movimentos...", moves.size());

        send_moves(moves);
    }

    void send_moves(const std::vector<std::string>& moves)
    {
        for (auto &m : moves) {
            auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            req->direction = m;

            RCLCPP_INFO(this->get_logger(), "Movendo: %s", m.c_str());
            move_client_->async_send_request(req);

            std::this_thread::sleep_for(150ms);
        }

        RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
    }

    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigator>());
    rclcpp::shutdown();
    return 0;
}
