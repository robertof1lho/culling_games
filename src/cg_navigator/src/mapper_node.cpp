#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

class MapperDFS : public rclcpp::Node {
public:
    MapperDFS()
    : Node("mapper_dfs_node"),
      H(40), W(40),
      r(30), c(30),
      have_sensor(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== DFS Mapper Iniciado ===");

        map.assign(H, std::vector<char>(W, '?'));
        visited.assign(H, std::vector<bool>(W, false));
        path.push_back({r, c});

        // robô inicia aqui
        map[r][c] = 'r';

        move_client =
            this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        sensor_sub =
            this->create_subscription<cg_interfaces::msg::RobotSensors>(
                "/culling_games/robot_sensors",
                10,
                std::bind(&MapperDFS::on_sensor, this, std::placeholders::_1)
            );

        // DFS passo a passo
        step_timer =
            this->create_wall_timer(150ms, std::bind(&MapperDFS::step, this));
    }

private:

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

    void update(int rr, int cc, const std::string &val)
    {
        if (!inside(rr, cc)) return;
        if (val.empty()) return;

        char v = val[0];

        // Atualiza se era desconhecido ou se um alvo foi detectado
        if (map[rr][cc] == '?' || v == 't') {
            map[rr][cc] = v;
            RCLCPP_INFO(this->get_logger(),
                "[MAP] Descoberto (%d,%d) = %c", rr, cc, v);
        }
    }

    bool inside(int rr, int cc) {
        return rr >= 0 && rr < H && cc >= 0 && cc < W;
    }

    bool free_or_target(char v) {
        return (v == 'f' || v == 't');
    }

    // Passo do DFS
    void step()
    {
        if (!have_sensor)
            return;

        have_sensor = false;

        // Se o robô pisou em 't', acabou!
        if (map[r][c] == 't') {
            RCLCPP_WARN(this->get_logger(),
                "=== OBJETIVO ALCANÇADO EM (%d,%d) ===", r, c);
            print_map();
            rclcpp::shutdown();
            return;
        }

        visited[r][c] = true;

        struct Move { int dr, dc; const char* name; };
        std::vector<Move> moves = {
            {-1,0,"up"}, {0,1,"right"}, {1,0,"down"}, {0,-1,"left"}
        };

        // Se o último sensor viu 't' em um vizinho cardinal, vá direto
        for (auto &m : moves) {
            int nr = r + m.dr;
            int nc = c + m.dc;

            if (!inside(nr, nc))
                continue;

            char sensed = 0;
            if (m.name == std::string("up") && !last.up.empty()) sensed = last.up[0];
            if (m.name == std::string("right") && !last.right.empty()) sensed = last.right[0];
            if (m.name == std::string("down") && !last.down.empty()) sensed = last.down[0];
            if (m.name == std::string("left") && !last.left.empty()) sensed = last.left[0];

            if (sensed == 't' || map[nr][nc] == 't') {
                RCLCPP_WARN(this->get_logger(),
                    "[DFS] ENTRANDO NO ALVO (%d,%d) via %s",
                    nr, nc, m.name);

                send_move(m.name);
                r = nr;
                c = nc;
                return;
            }
        }

        // Priorizar alvo se ele aparecer em qualquer vizinho
        for (auto &m : moves) {
            int nr = r + m.dr;
            int nc = c + m.dc;

            if (!inside(nr, nc))
                continue;

            if (map[nr][nc] == 't') {
                RCLCPP_WARN(this->get_logger(),
                    "[DFS] ENTRANDO NO ALVO (%d,%d) via %s",
                    nr, nc, m.name);

                send_move(m.name);
                r = nr;
                c = nc;
                return;
            }
        }

        // --------------------------------------------------------
        // EXPANDIR (andar para uma célula livre não visitada)
        // --------------------------------------------------------
        for (auto &m : moves) {

            int nr = r + m.dr;
            int nc = c + m.dc;

            if (!inside(nr, nc))
                continue;

            char cell = map[nr][nc];

            // parede = ignora
            if (cell == 'b')
                continue;

            // desconhecido = esperar sensor atualizar
            if (cell == '?')
                continue;

            // já visitado e não é alvo = ignora
            if (visited[nr][nc] && cell != 't')
                continue;


            // movimento normal
            send_move(m.name);

            r = nr;
            c = nc;

            RCLCPP_INFO(this->get_logger(),
                "[DFS] Avançou para (%d,%d) via %s", r, c, m.name);

            path.push_back({r, c});
            return;
        }

        if (path.size() > 1) {
            auto prev = path[path.size() - 2];
            int pr = prev.first;
            int pc = prev.second;

            std::string back = reverse_dir(r, c, pr, pc);

            send_move(back);

            RCLCPP_INFO(this->get_logger(),
                "[DFS] Voltando para (%d,%d)", pr, pc);

            r = pr;
            c = pc;
            path.pop_back();
        }
    }

    void send_move(const std::string &dir)
    {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;

        move_client->async_send_request(req);

        std::this_thread::sleep_for(130ms);
    }

    std::string reverse_dir(int r1, int c1, int r2, int c2)
    {
        if (r2 == r1 - 1) return "up";
        if (r2 == r1 + 1) return "down";
        if (c2 == c1 - 1) return "left";
        if (c2 == c1 + 1) return "right";
        return "";
    }

    void print_map()
    {
        std::cout << "\n===== MAPA FINAL =====\n";
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++)
                std::cout << map[i][j];
            std::cout << "\n";
        }
    }


    int H, W;
    int r, c;
    bool have_sensor;

    std::vector<std::vector<char>> map;
    std::vector<std::vector<bool>> visited;
    std::vector<std::pair<int,int>> path;

    cg_interfaces::msg::RobotSensors last;

    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub;
    rclcpp::TimerBase::SharedPtr step_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperDFS>());
    rclcpp::shutdown();
    return 0;
}
