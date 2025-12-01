// ======================================================================
//  MAPPER DFS + RESET + BFS FINAL
// ======================================================================

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"

#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <queue>

using namespace std::chrono_literals;


// ======================================================================
// BFS — encontra caminho no mapa
// ======================================================================
std::vector<std::pair<int,int>> bfs_path(
    const std::vector<std::vector<char>>& grid,
    int H, int W,
    int sr, int sc,
    int tr, int tc)
{
    std::queue<std::pair<int,int>> q;
    std::vector<std::vector<bool>> visited(H, std::vector<bool>(W, false));
    std::vector<std::vector<std::pair<int,int>>> parent(
        H, std::vector<std::pair<int,int>>(W, {-1, -1}));

    auto free_cell = [&](int r, int c){
        return grid[r][c] == 'f' || grid[r][c] == 'r' || grid[r][c] == 't';
    };

    visited[sr][sc] = true;
    q.push({sr, sc});

    int dr[4] = {-1, 1, 0, 0};
    int dc[4] = { 0, 0,-1, 1};

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

    if (!visited[tr][tc])
        return {};

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


// Converter caminho para lista de movimentos
std::vector<std::string> path_to_moves(
    const std::vector<std::pair<int,int>>& path)
{
    std::vector<std::string> moves;

    for (int i = 0; i < (int)path.size() - 1; i++) {
        int r1 = path[i].first,  c1 = path[i].second;
        int r2 = path[i+1].first, c2 = path[i+1].second;

        if (r2 == r1 - 1) moves.push_back("up");
        else if (r2 == r1 + 1) moves.push_back("down");
        else if (c2 == c1 - 1) moves.push_back("left");
        else if (c2 == c1 + 1) moves.push_back("right");
    }

    return moves;
}

class MapperDFS : public rclcpp::Node {
public:
    MapperDFS()
    : Node("mapper_dfs_node"),
      H(60), W(60),
      start_r(15), start_c(15),
      r(15), c(15),
      target_r(-1), target_c(-1),
      have_sensor(false),
      mapping_completed(false),
      awaiting_reset(false)
    {
        map.assign(H, std::vector<char>(W, '?'));
        visited.assign(H, std::vector<bool>(W, false));
        map[r][c] = 'r';
        path.push_back({r, c});

        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

        sensor_sub =
            this->create_subscription<cg_interfaces::msg::RobotSensors>(
                "/culling_games/robot_sensors",
                sensor_qos,
                std::bind(&MapperDFS::on_sensor, this, std::placeholders::_1)
            );

        move_client =
            this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        reset_client =
            this->create_client<cg_interfaces::srv::Reset>("/reset");

        step_timer =
            this->create_wall_timer(2ms, std::bind(&MapperDFS::step, this));
    }

private:

// Callback do Sensor
void on_sensor(const cg_interfaces::msg::RobotSensors::SharedPtr msg)
{
    have_sensor = true;

    update(r - 1, c,     msg->up);
    update(r + 1, c,     msg->down);
    update(r,     c - 1, msg->left);
    update(r,     c + 1, msg->right);

    update(r - 1, c - 1, msg->up_left);
    update(r - 1, c + 1, msg->up_right);
    update(r + 1, c - 1, msg->down_left);
    update(r + 1, c + 1, msg->down_right);

    last = *msg;
}

// Atualiza o mapa
void update(int rr, int cc, const std::string &val)
{
    if (!inside(rr, cc)) return;
    if (val.empty()) return;

    char v = val[0];

    if (v == 't') {
        if (target_r == -1) {
            target_r = rr;
            target_c = cc;
            RCLCPP_WARN(this->get_logger(),
                "[MAP] ALVO DETECTADO EM (%d,%d)", rr, cc);
        }
        map[rr][cc] = 't';
        // Marca como visitado para o DFS ignorar completamente o alvo
        visited[rr][cc] = true;
        return;
    }

    if (map[rr][cc] == '?') {
        map[rr][cc] = v;
    }
}

void step()
{
    if (!have_sensor)
        return;

    have_sensor = false;

    if (awaiting_reset) {
        awaiting_reset = false;
        r = start_r;
        c = start_c;
        visited.assign(H, std::vector<bool>(W, false));
        path = {{r, c}};

        RCLCPP_WARN(this->get_logger(),
            "RESET concluído — começando BFS final!");

        run_bfs_and_execute();
        return;
    }

    // DFS 
    visited[r][c] = true;

    struct Move { int dr, dc; const char* name; };
    std::vector<Move> moves = {
        {-1,0,"up"}, {0,1,"right"}, {1,0,"down"}, {0,-1,"left"}
    };

    for (auto &m : moves) {
        int nr = r + m.dr;
        int nc = c + m.dc;

        if (!inside(nr, nc)) continue;
        if (target_r != -1 && nr == target_r && nc == target_c) continue;
        char cell = map[nr][nc];

        if (cell == 't') continue;
        if (cell == 'b') continue;
        if (cell == '?') continue;
        if (visited[nr][nc]) continue;

        send_move(m.name);
        r = nr; c = nc;

        RCLCPP_INFO(this->get_logger(),
            "[DFS] Avançou para (%d,%d) via %s", r, c, m.name);

        path.push_back({r, c});
        return;
    }

    // Backtracking
    if (path.size() > 1) {
        auto prev = path[path.size() - 2];
        int pr = prev.first, pc = prev.second;
        std::string back = reverse_dir(r, c, pr, pc);
        send_move(back);

        r = pr;
        c = pc;

        path.pop_back();
        return;
    }

    // ------------ mapeamento completo -------------
    mapping_completed = true;

    RCLCPP_WARN(this->get_logger(),
        "=== MAPEAMENTO COMPLETO! RESETANDO LABIRINTO ===");

    call_reset();
}



// ======================================================================
// RESET
// ======================================================================
void call_reset()
{
    auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
    req->is_random = false;

    reset_client->async_send_request(req);
    awaiting_reset = true;
}



// ======================================================================
// BFS FINAL
// ======================================================================
    void run_bfs_and_execute()
{
    if (target_r == -1) {
        RCLCPP_ERROR(this->get_logger(),
            "ERRO: alvo não detectado durante o mapeamento!");
        print_map();
        return;
    }

    auto bfs_map = map;

    for (int i = 0; i < H; i++)
        for (int j = 0; j < W; j++)
            if (bfs_map[i][j] == '?')
                bfs_map[i][j] = 'b';

    auto path_b = bfs_path(bfs_map, H, W, start_r, start_c, target_r, target_c);
    if (path_b.empty()) {
        RCLCPP_ERROR(this->get_logger(),
            "BFS não encontrou caminho!");
        print_map();
        return;
    }

    auto moves = path_to_moves(path_b);

    RCLCPP_WARN(this->get_logger(),
        "[BFS] Execução final iniciada com %ld movimentos", moves.size());

    for (auto &m : moves) {
        send_move(m);
        std::this_thread::sleep_for(10ms);
    }

    RCLCPP_WARN(this->get_logger(),
        "=== ALVO ALCANÇADO ===");
    print_map();
    rclcpp::shutdown();
}



// ======================================================================
// Helpers
// ======================================================================
bool inside(int rr, int cc) {
    return rr >= 0 && rr < H && cc >= 0 && cc < W;
}

std::string reverse_dir(int r1, int c1, int r2, int c2)
{
    if (r2 == r1 - 1) return "up";
    if (r2 == r1 + 1) return "down";
    if (c2 == c1 - 1) return "left";
    if (c2 == c1 + 1) return "right";
    return "";
}

void send_move(const std::string &dir)
{
    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = dir;
    move_client->async_send_request(req);
}

void print_map()
{
    RCLCPP_INFO(this->get_logger(), "\n===== MAPA FINAL =====");

    for (int i = 0; i < H; i++) {
        std::string linha;
        for (int j = 0; j < W; j++)
            linha.push_back(map[i][j]);
        RCLCPP_INFO(this->get_logger(), "%s", linha.c_str());
    }
}



// ======================================================================
// Variáveis internas
// ======================================================================
int H, W;
int start_r, start_c;
int r, c;

int target_r, target_c;

bool have_sensor;
bool mapping_completed;
bool awaiting_reset;

std::vector<std::vector<char>> map;
std::vector<std::vector<bool>> visited;
std::vector<std::pair<int,int>> path;

cg_interfaces::msg::RobotSensors last;

rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client;
rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client;
rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub;
rclcpp::TimerBase::SharedPtr step_timer;
};




// ======================================================================
// MAIN
// ======================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapperDFS>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
