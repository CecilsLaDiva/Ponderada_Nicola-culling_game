#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <set>
#include <tuple>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/reset.hpp"

using namespace std::chrono_literals;

// Estrutura simples para coordenadas
struct Point {
    int r, c;
    bool operator<(const Point& other) const {
        return std::tie(r, c) < std::tie(other.r, other.c);
    }
    bool operator==(const Point& other) const {
        return r == other.r && c == other.c;
    }
};

class MazeResolver : public rclcpp::Node {
public:
    MazeResolver() : Node("maze_resolver_cpp") {
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        reset_client_ = this->create_client<cg_interfaces::srv::Reset>("/reset");
    }

    void wait_for_services() {
        while (!move_client_->wait_for_service(1s) || 
               !map_client_->wait_for_service(1s) || 
               !reset_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Aguardando serviços...");
        }
    }

    void run() {
        wait_for_services();

        // 1. Resetar Jogo
        RCLCPP_INFO(this->get_logger(), "Resetando o labirinto...");
        auto reset_req = std::make_shared<cg_interfaces::srv::Reset::Request>();
        reset_req->is_random = true;
        
        auto reset_future = reset_client_->async_send_request(reset_req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), reset_future) == rclcpp::FutureReturnCode::SUCCESS) {
             RCLCPP_INFO(this->get_logger(), "Mapa carregado: %s", reset_future.get()->loaded_map_name.c_str());
        }
        std::this_thread::sleep_for(1s);

        // 2. Pegar Mapa
        RCLCPP_INFO(this->get_logger(), "Lendo o mapa...");
        auto map_req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto map_future = map_client_->async_send_request(map_req);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), map_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao pegar mapa");
            return;
        }

        auto map_res = map_future.get();
        int rows = map_res->occupancy_grid_shape[0];
        int cols = map_res->occupancy_grid_shape[1];
        auto flat_grid = map_res->occupancy_grid_flattened;

        // Reconstruir Grid
        std::vector<std::vector<std::string>> grid(rows, std::vector<std::string>(cols));
        Point start_pos = {-1, -1};
        Point target_pos = {-1, -1};

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                std::string val = flat_grid[r * cols + c];
                grid[r][c] = val;
                if (val == "r") start_pos = {r, c};
                else if (val == "t") target_pos = {r, c};
            }
        }

        if (start_pos.r != -1 && target_pos.r != -1) {
            auto path = bfs(grid, start_pos, target_pos, rows, cols);
            if (!path.empty()) {
                RCLCPP_INFO(this->get_logger(), "Executando movimentos...");
                for (const auto& step : path) {
                    move_robot(step);
                    std::this_thread::sleep_for(100ms);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Caminho não encontrado!");
            }
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;

    void move_robot(std::string direction) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = direction;
        auto future = move_client_->async_send_request(req);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }

    std::vector<std::string> bfs(const std::vector<std::vector<std::string>>& grid, Point start, Point target, int rows, int cols) {
        std::deque<std::pair<Point, std::vector<std::string>>> queue;
        queue.push_back({start, {}});
        
        std::set<Point> visited;
        visited.insert(start);

        // Baixo, Direita, Cima, Esquerda
        std::vector<std::tuple<int, int, std::string>> directions = {
            {1, 0, "down"}, {0, 1, "right"}, {-1, 0, "up"}, {0, -1, "left"}
        };

        int nodes_explored = 0;

        while (!queue.empty()) {
            auto [curr, path] = queue.front();
            queue.pop_front();
            nodes_explored++;

            if (curr == target) {
                RCLCPP_INFO(this->get_logger(), "--- RELATÓRIO BFS ---");
                RCLCPP_INFO(this->get_logger(), "Passos: %ld", path.size());
                RCLCPP_INFO(this->get_logger(), "Esforço: %d", nodes_explored);
                return path;
            }

            for (auto [dr, dc, move_name] : directions) {
                int nr = curr.r + dr;
                int nc = curr.c + dc;

                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                    if (grid[nr][nc] != "b" && visited.find({nr, nc}) == visited.end()) {
                        visited.insert({nr, nc});
                        auto new_path = path;
                        new_path.push_back(move_name);
                        queue.push_back({{nr, nc}, new_path});
                    }
                }
            }
        }
        return {};
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeResolver>();
    node->run();
    rclcpp::shutdown();
    return 0;
}