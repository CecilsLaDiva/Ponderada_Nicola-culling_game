#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <set>
#include <tuple>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"

using namespace std::chrono_literals;

struct Point {
    int r, c;
    bool operator<(const Point& other) const { return std::tie(r, c) < std::tie(other.r, other.c); }
    bool operator==(const Point& other) const { return r == other.r && c == other.c; }
};

class MazeSemAlvoPt2 : public rclcpp::Node {
public:
    MazeSemAlvoPt2() : Node("maze_semalvopt2_cpp") {
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        reset_client_ = this->create_client<cg_interfaces::srv::Reset>("/reset");
        sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10, std::bind(&MazeSemAlvoPt2::sensor_callback, this, std::placeholders::_1));
        
        rows_ = 29; cols_ = 29;
        internal_map_.assign(rows_, std::vector<std::string>(cols_, "?"));
        internal_map_[1][1] = "f";
    }

    void run() {
        wait_for_services();
        RCLCPP_INFO(this->get_logger(), "Iniciando Exploração CEGA (C++)...");
        reset_game();
        wait_for_sensor();

        robot_pos_ = {1, 1};
        visited_.insert(robot_pos_);
        stack_.push_back(robot_pos_);

        Point target_found_pos = {-1, -1};

        while (!stack_.empty()) {
            latest_sensors_ = nullptr;
            wait_for_sensor();
            update_map();

            int curr_r = robot_pos_.r;
            int curr_c = robot_pos_.c;

            if (internal_map_[curr_r][curr_c] == "t") {
                target_found_pos = robot_pos_;
                break;
            }

            bool nearby_target = false;
            std::vector<std::tuple<std::string, int, int>> dirs = {
                {"down", 1, 0}, {"right", 0, 1}, {"up", -1, 0}, {"left", 0, -1}
            };
            
            for (auto [move, dr, dc] : dirs) {
                int nr = curr_r + dr; int nc = curr_c + dc;
                if (is_valid(nr, nc) && internal_map_[nr][nc] == "t") {
                    move_robot(move);
                    target_found_pos = {nr, nc};
                    nearby_target = true;
                    break;
                }
            }
            if (nearby_target) break;

            bool moved = false;
            for (auto [move, dr, dc] : dirs) {
                int nr = curr_r + dr; int nc = curr_c + dc;
                if (is_valid(nr, nc)) {
                    std::string val = internal_map_[nr][nc];
                    if ((val == "f" || val == "t") && visited_.find({nr, nc}) == visited_.end()) {
                        if (move_robot(move)) {
                            visited_.insert({nr, nc});
                            stack_.push_back({curr_r, curr_c});
                            std::this_thread::sleep_for(50ms);
                            moved = true;
                            break;
                        }
                    }
                }
            }

            if (!moved) {
                if (!stack_.empty()) {
                    Point back = stack_.back();
                    stack_.pop_back();
                    
                    int dr = back.r - curr_r;
                    int dc = back.c - curr_c;
                    std::string back_move = "";
                    if (dr == 1) back_move = "down";
                    else if (dr == -1) back_move = "up";
                    else if (dc == 1) back_move = "right";
                    else if (dc == -1) back_move = "left";

                    if (!back_move.empty()) {
                        move_robot(back_move);
                        std::this_thread::sleep_for(50ms);
                    }
                }
            }
        }

        if (target_found_pos.r != -1) {
            RCLCPP_INFO(this->get_logger(), "ALVO ENCONTRADO! Rodando BFS de validação...");
            auto path = bfs_check({1,1}, target_found_pos);
            if (!path.empty()) {
                RCLCPP_INFO(this->get_logger(), "SUCESSO! Mapa válido. Rota de %ld passos.", path.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "ERRO: Mapa inválido.");
            }
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    
    cg_interfaces::msg::RobotSensors::SharedPtr latest_sensors_;
    Point robot_pos_;
    std::vector<Point> stack_;
    std::set<Point> visited_;
    int rows_, cols_;
    std::vector<std::vector<std::string>> internal_map_;

    void wait_for_services() {
        while (!move_client_->wait_for_service(1s) || !reset_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Aguardando serviços...");
        }
    }

    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        latest_sensors_ = msg;
    }

    void wait_for_sensor() {
        while (latest_sensors_ == nullptr && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(10ms);
        }
    }

    void reset_game() {
        auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
        req->is_random = true;
        auto future = reset_client_->async_send_request(req);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }

    bool move_robot(std::string direction) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = direction;
        auto future = move_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto res = future.get();
            if (res->success) {
                robot_pos_ = {res->robot_pos[0], res->robot_pos[1]};
                return true;
            }
        }
        return false;
    }

    bool is_valid(int r, int c) {
        return r >= 0 && r < rows_ && c >= 0 && c < cols_;
    }

    void update_map() {
        if (!latest_sensors_) return;
        int r = robot_pos_.r; 
        int c = robot_pos_.c;
        
        std::vector<std::tuple<std::string, int, int>> deltas = {
            {latest_sensors_->up, -1, 0}, {latest_sensors_->down, 1, 0},
            {latest_sensors_->left, 0, -1}, {latest_sensors_->right, 0, 1}
        };

        for (auto [val, dr, dc] : deltas) {
            int nr = r + dr; int nc = c + dc;
            if (is_valid(nr, nc)) {
                if (internal_map_[nr][nc] == "?") internal_map_[nr][nc] = val;
            }
        }
    }

    std::vector<std::string> bfs_check(Point start, Point target) {
        std::deque<std::pair<Point, std::vector<std::string>>> queue;
        queue.push_back({start, {}});
        std::set<Point> visited_bfs;
        visited_bfs.insert(start);

        std::vector<std::tuple<int, int, std::string>> dirs = {{1,0,"down"}, {0,1,"right"}, {-1,0,"up"}, {0,-1,"left"}};

        while(!queue.empty()) {
            auto [curr, path] = queue.front(); queue.pop_front();
            if (curr == target) return path;

            for(auto [dr, dc, move] : dirs) {
                int nr = curr.r + dr; int nc = curr.c + dc;
                if(is_valid(nr, nc)) {
                    std::string cell = internal_map_[nr][nc];
                    if ((cell == "f" || cell == "t" || cell == "r") && visited_bfs.find({nr,nc}) == visited_bfs.end()) {
                        visited_bfs.insert({nr,nc});
                        auto new_p = path; new_p.push_back(move);
                        queue.push_back({{nr,nc}, new_p});
                    }
                }
            }
        }
        return {};
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeSemAlvoPt2>();
    node->run();
    rclcpp::shutdown();
    return 0;
}