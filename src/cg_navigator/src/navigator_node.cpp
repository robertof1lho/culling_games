#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <vector>
#include <queue>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// Transforma vetor flat do labirinto em matrix
std::vector<std::vector<char>> flat_to_matrix(
    const std::vector<std::string> &flat, int W, int H)
{
    // Variável que armazena uma matriz
    std::vector<std::vector<char>> grid(H, std::vector<char>(W));

    // Monta valores da matriz de acordo como vetor flat do mapa
    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++)
            grid[r][c] = flat[r * W + c][0];
                        
    return grid;
}

// Algoritmo de breath firts search
std::vector<std::pair<int,int>> bfs_path(
    const std::vector<std::vector<char>>& grid,
    int H, int W,
    int sr, int sc,
    int tr, int tc)
{
    std::queue<std::pair<int,int>> q;

    std::vector<std::vector<bool>> visited(H, std::vector<bool>(W, false));

    std::vector<std::vector<std::pair<int,int>>> parent(
        H, std::vector<std::pair<int,int>>(W, {-1, -1})
    );

    auto free_cell = [&](int r, int c){
        return grid[r][c] == 'f' || grid[r][c] == 'r' || grid[r][c] == 't';
    };

    visited[sr][sc] = true;
    q.push({sr, sc});

    int dr[4] = {-1, 1, 0, 0};
    int dc[4] = {0, 0, -1, 1};

    while (!q.empty()) {
        auto [r, c] = q.front(); q.pop();

        if (r == tr && c == tc)
            break;

        for (int i = 0; i < 4; i++) {
            int nr = r + dr[i];
            int nc = c + dc[i];

            if (nr >= 0 && nr < H &&
                nc >= 0 && nc < W &&
                !visited[nr][nc] &&
                free_cell(nr, nc))
            {
                visited[nr][nc] = true;
                parent[nr][nc] = {r, c};
                q.push({nr, nc});
            }
        }
    }

    if (!visited[tr][tc]) return {};

    std::vector<std::pair<int,int>> path;
    int r = tr, c = tc;

    while (!(r == -1 && c == -1)) {
        path.push_back({r, c});
        auto p = parent[r][c];
        r = p.first;
        c = p.second;
    }

    std::reverse(path.begin(), path.end());
    return path;
}


// Converte solução do caminho para movimentos do robo
std::vector<std::string> path_to_moves(
    const std::vector<std::pair<int,int>>& path)
{
    std::vector<std::string> moves;

    for (int i = 0; i < (int)path.size() - 1; i++) {
        int r1 = path[i].first;
        int c1 = path[i].second;
        int r2 = path[i+1].first;
        int c2 = path[i+1].second;

        if (r2 == r1 - 1) moves.push_back("up");
        else if (r2 == r1 + 1) moves.push_back("down");
        else if (c2 == c1 - 1) moves.push_back("left");
        else if (c2 == c1 + 1) moves.push_back("right");
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

        int sr = -1, sc = -1;
        int tr = -1, tc = -1;
    
        for (int r = 0; r < H; r++){
            for (int c = 0; c < W; c++){
                if (grid[r][c] == 'r') {   
                    sr = r;
                    sc = c;
                } else if (grid[r][c] == 't') {
                    tr = r;
                    tc = c;
                }
            }
        }

        if (sr == -1 || sc == -1 || tr == -1 || tc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Start ou Goal não encontrados!");
            return;
        }

        auto path = bfs_path(grid, H, W, sr, sc, tr, tc);

        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "SEM CAMINHO!");
            return;
        }

        auto moves = path_to_moves(path);
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
    auto node = std::make_shared<Navigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
