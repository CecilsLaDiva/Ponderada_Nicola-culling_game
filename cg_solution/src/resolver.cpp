#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/reset.hpp>
#include <queue>
#include <vector>
#include <string>
#include <set>
#include <thread>
#include <iostream>

using namespace std;
using namespace rclcpp;
using namespace std::chrono_literals;

string grid_global[100][100];
int linhas_g, colunas_g;

class MazeResolver : public Node {
public:
    Client<cg_interfaces::srv::MoveCmd>::SharedPtr cl_move;
    Client<cg_interfaces::srv::GetMap>::SharedPtr cl_map;
    Client<cg_interfaces::srv::Reset>::SharedPtr cl_reset;

    MazeResolver() : Node("maze_resolver") {
        cl_move = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        cl_map = create_client<cg_interfaces::srv::GetMap>("/get_map");
        cl_reset = create_client<cg_interfaces::srv::Reset>("/reset");
        

        while (!cl_move->wait_for_service(1s)) cout << "aguardando move..." << endl;
        while (!cl_map->wait_for_service(1s)) cout << "aguardando map..." << endl;
        while (!cl_reset->wait_for_service(1s)) cout << "aguardando reset..." << endl;
    }

    void do_reset() {
        auto req = make_shared<cg_interfaces::srv::Reset::Request>();
        req->is_random = true;
        req->map_name = "";
        auto f = cl_reset->async_send_request(req);
        spin_until_future_complete(this->shared_from_this(), f);
    }

    void do_move(string d) {
        auto req = make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = d;
        auto f = cl_move->async_send_request(req);
        spin_until_future_complete(this->shared_from_this(), f);
    }

    vector<string> bfs(pair<int,int> inicio, pair<int,int> alvo) {
        queue<pair<pair<int,int>, vector<string>>> q;
        set<pair<int,int>> visitados;
        
        vector<string> caminho_vazio;
        q.push({inicio, caminho_vazio});
        visitados.insert(inicio);
        
        int dr[] = {1, 0, -1, 0};
        int dc[] = {0, 1, 0, -1};
        string moves[] = {"down", "right", "up", "left"};

        while(!q.empty()) {
            auto item = q.front();
            q.pop();
            
            int r = item.first.first;
            int c = item.first.second;
            vector<string> path = item.second;

            if(r == alvo.first && c == alvo.second) {
                cout << "Caminho encontrado! Passos: " << path.size() << endl;
                return path;
            }

            for(int i=0; i<4; i++) {
                int nr = r + dr[i];
                int nc = c + dc[i];
                
                if(nr >= 0 && nr < linhas_g && nc >= 0 && nc < colunas_g) {
                    if(grid_global[nr][nc] != "b" && visitados.count({nr, nc}) == 0) {
                        visitados.insert({nr, nc});
                        vector<string> new_path = path;
                        new_path.push_back(moves[i]);
                        q.push({{nr, nc}, new_path});
                    }
                }
            }
        }
        return {};
    }

    void start() {
        cout << "Resetando..." << endl;
        do_reset();
        this_thread::sleep_for(500ms);

        auto req_map = make_shared<cg_interfaces::srv::GetMap::Request>();
        auto fut_map = cl_map->async_send_request(req_map);
        spin_until_future_complete(this->shared_from_this(), fut_map);
        auto res = fut_map.get();

        linhas_g = res->occupancy_grid_shape[0];
        colunas_g = res->occupancy_grid_shape[1];
        auto flat = res->occupancy_grid_flattened;

        pair<int,int> start_pos;
        pair<int,int> target_pos;

        int k=0;
        for(int i=0; i<linhas_g; i++) {
            for(int j=0; j<colunas_g; j++) {
                string val = flat[k];
                grid_global[i][j] = val;
                if(val == "r") start_pos = {i, j};
                if(val == "t") target_pos = {i, j};
                k++;
            }
        }

        auto rota = bfs(start_pos, target_pos);
        
        if(rota.size() > 0) {
            cout << "Executando..." << endl;
            for(size_t i=0; i<rota.size(); i++) {
                do_move(rota[i]);
                this_thread::sleep_for(100ms);
            }
        } else {
            cout << "Erro: BFS nao achou nada." << endl;
        }
    }
};

int main(int argc, char **argv) {
    init(argc, argv);
    auto node = make_shared<MazeResolver>();
    node->start();
    shutdown();
    return 0;
}