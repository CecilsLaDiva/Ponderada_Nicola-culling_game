#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/reset.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <deque>
#include <vector>
#include <string>
#include <set>
#include <thread>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <tuple>

using namespace std;
using namespace rclcpp;


const int ALVO_R = 14;
const int ALVO_C = 14;

class MazeResolverPt2 : public Node {
public:
    Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_cli;
    Client<cg_interfaces::srv::Reset>::SharedPtr reset_cli;
    Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub;
    
    cg_interfaces::msg::RobotSensors::SharedPtr last_sensor;
    
    string my_map[29][29];
    int r_pos, c_pos;

    MazeResolverPt2() : Node("maze_resolverpt2") {
        move_cli = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        reset_cli = create_client<cg_interfaces::srv::Reset>("/reset");
        
        sensor_sub = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10,
            [this](cg_interfaces::msg::RobotSensors::SharedPtr msg){
                this->last_sensor = msg;
            });
            
        for(int i=0; i<29; i++) for(int j=0; j<29; j++) my_map[i][j] = "?";
        my_map[1][1] = "f";
        r_pos = 1; c_pos = 1;
        
        while(!move_cli->wait_for_service(1s)) cout << ".";
        while(!reset_cli->wait_for_service(1s)) cout << ".";
        cout << " Ready!" << endl;
    }

    bool call_move(string d) {
        auto req = make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = d;
        auto f = move_cli->async_send_request(req);
        spin_until_future_complete(this->shared_from_this(), f);
        auto res = f.get();
        if(res->success) {
            r_pos = res->robot_pos[0];
            c_pos = res->robot_pos[1];
        }
        return res->success;
    }

    void update_sensors() {
        if(!last_sensor) return;
        if(r_pos > 0 && my_map[r_pos-1][c_pos] == "?") my_map[r_pos-1][c_pos] = last_sensor->up;
        if(r_pos < 28 && my_map[r_pos+1][c_pos] == "?") my_map[r_pos+1][c_pos] = last_sensor->down;
        if(c_pos > 0 && my_map[r_pos][c_pos-1] == "?") my_map[r_pos][c_pos-1] = last_sensor->left;
        if(c_pos < 28 && my_map[r_pos][c_pos+1] == "?") my_map[r_pos][c_pos+1] = last_sensor->right;
    }

    vector<string> validate_map(int tr, int tc) {
        queue<pair<pair<int,int>, vector<string>>> q;
        set<pair<int,int>> vis;
        q.push({{1,1}, {}});
        vis.insert({1,1});
        
        int dr[]={1, 0, -1, 0}; 
        int dc[]={0, -1, 0, 1};
        string n[]={"down", "left", "up", "right"};
        
        while(!q.empty()){
            auto curr = q.front(); q.pop();
            int r=curr.first.first; int c=curr.first.second;
            if(r==tr && c==tc) return curr.second;
            
            for(int i=0;i<4;i++){
                int nr=r+dr[i]; int nc=c+dc[i];
                if(nr>=0 && nr<29 && nc>=0 && nc<29){
                    string val = my_map[nr][nc];
                    if((val=="f"||val=="t"||val=="r") && vis.count({nr,nc})==0){
                        vis.insert({nr,nc});
                        vector<string> p=curr.second; p.push_back(n[i]);
                        q.push({{nr,nc}, p});
                    }
                }
            }
        }
        return {};
    }

    void run_logic() {
        auto r_req = make_shared<cg_interfaces::srv::Reset::Request>();
        r_req->is_random = true;
        auto rf = reset_cli->async_send_request(r_req);
        spin_until_future_complete(this->shared_from_this(), rf);
        cout << "Mapa: " << rf.get()->loaded_map_name << endl;

        while(!last_sensor) { spin_some(this->shared_from_this()); this_thread::sleep_for(100ms); }

        vector<pair<int,int>> stack;
        set<pair<int,int>> visited;
        
        visited.insert({r_pos, c_pos});
        stack.push_back({r_pos, c_pos});

        int final_r = -1, final_c = -1;
        bool found = false;

        while(!stack.empty()) {
            last_sensor = nullptr;
            while(!last_sensor) { spin_some(this->shared_from_this()); this_thread::sleep_for(50ms); }
            update_sensors();

            if(my_map[r_pos][c_pos] == "t") {
                final_r=r_pos; final_c=c_pos; found=true; break;
            }


            int drs[]={1, 0, -1, 0}; 
            int dcs[]={0, -1, 0, 1}; 
            string nms[]={"down", "left", "up", "right"};
            
            for(int i=0;i<4;i++){
                int nr=r_pos+drs[i]; int nc=c_pos+dcs[i];
                if(nr>=0 && nr<29 && nc>=0 && nc<29){
                    if(my_map[nr][nc]=="t"){
                        call_move(nms[i]);
                        final_r=nr; final_c=nc; found=true; vizinho_alvo=true;
                        break;
                    }
                }
            }
            if(vizinho_alvo) break;

            vector<tuple<int, string, int, int, int, int>> valid_moves;
            
            for(int i=0; i<4; i++) {
                int nr = r_pos + drs[i];
                int nc = c_pos + dcs[i];
                if(nr>=0 && nr<29 && nc>=0 && nc<29) {
                    string cell = my_map[nr][nc];
                    if((cell=="f" || cell=="t") && visited.count({nr, nc}) == 0) {
                        
                        int dist_base = abs(nr - ALVO_R) + abs(nc - ALVO_C);
                        
                        int penalidade = 0;
                        if(nms[i] == "down") penalidade = 0;      
                        else if(nms[i] == "left") penalidade = 1; 
                        else if(nms[i] == "up") penalidade = 2;
                        else if(nms[i] == "right") penalidade = 3;
                        
                        int score_final = (dist_base * 10) + penalidade;

                        valid_moves.push_back({score_final, nms[i], drs[i], dcs[i], nr, nc});
                    }
                }
            }

            sort(valid_moves.begin(), valid_moves.end());

            if(valid_moves.size() > 0) {
                auto best = valid_moves[0];
                string best_move = get<1>(best); 
                int nr = get<4>(best); 
                int nc = get<5>(best); 
                
                int dr_used = get<2>(best);
                int dc_used = get<3>(best);

                if(call_move(best_move)) {
                    visited.insert({nr, nc});
                    stack.push_back({r_pos-dr_used, c_pos-dc_used}); 
                    this_thread::sleep_for(50ms);
                }
            } else {
                if(!stack.empty()) {
                    auto back_pos = stack.back();
                    stack.pop_back();
                    
                    int diff_r = back_pos.first - r_pos;
                    int diff_c = back_pos.second - c_pos;
                    
                    string b_move = "";
                    if(diff_r == -1) b_move = "down";
                    else if(diff_r == 1) b_move = "up";
                    else if(diff_c == 1) b_move = "right";
                    else if(diff_c == -1) b_move = "left";
                    
                    if(b_move != "") {
                        call_move(b_move);
                        this_thread::sleep_for(50ms);
                    }
                } else break;
            }
        }

        if(found) {
            cout << "--- ALVO ENCONTRADO! ---" << endl;
            auto path = validate_map(final_r, final_c);
            if(path.size() > 0) cout << "SUCESSO! Mapa valido. Passos: " << path.size() << endl;
            else cout << "ERRO: Caminho nao bate." << endl;
        } else {
            cout << "Desisto, nao achei." << endl;
        }
    }
};

int main(int argc, char **argv) {
    init(argc, argv);
    auto n = make_shared<MazeResolverPt2>();
    n->run_logic();
    shutdown();
    return 0;
}