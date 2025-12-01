#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/reset.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <deque>
#include <vector>
#include <string>
#include <set>
#include <thread>
#include <iostream>

using namespace std;
using namespace rclcpp;

class MazeSemAlvoPt2 : public Node {
public:
    Client<cg_interfaces::srv::MoveCmd>::SharedPtr cl_move;
    Client<cg_interfaces::srv::Reset>::SharedPtr cl_reset;
    Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sub;
    cg_interfaces::msg::RobotSensors::SharedPtr dados_sensor;

    string mapa[29][29];
    int x, y; // x=linha, y=coluna

    MazeSemAlvoPt2() : Node("maze_semalvopt2") {
        cl_move = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        cl_reset = create_client<cg_interfaces::srv::Reset>("/reset");
        
        sub = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10,
            [this](cg_interfaces::msg::RobotSensors::SharedPtr m){ dados_sensor=m; });

        for(int i=0;i<29;i++) for(int j=0;j<29;j++) mapa[i][j]="?";
        mapa[1][1]="f";
        x=1; y=1;
    }

    bool andar(string dir) {
        auto r = make_shared<cg_interfaces::srv::MoveCmd::Request>();
        r->direction = dir;
        auto f = cl_move->async_send_request(r);
        spin_until_future_complete(this->shared_from_this(), f);
        auto res = f.get();
        if(res->success) {
            x = res->robot_pos[0];
            y = res->robot_pos[1];
        }
        return res->success;
    }

    void sensor_to_map() {
        if(!dados_sensor) return;
        if(x-1>=0 && mapa[x-1][y]=="?") mapa[x-1][y] = dados_sensor->up;
        if(x+1<29 && mapa[x+1][y]=="?") mapa[x+1][y] = dados_sensor->down;
        if(y-1>=0 && mapa[x][y-1]=="?") mapa[x][y-1] = dados_sensor->left;
        if(y+1<29 && mapa[x][y+1]=="?") mapa[x][y+1] = dados_sensor->right;
    }
    
    vector<string> verify(int tx, int ty) {
        deque<pair<pair<int,int>, vector<string>>> q;
        set<pair<int,int>> vs;
        q.push_back({{1,1}, {}}); vs.insert({1,1});
        
        int dx[]={1,0,-1,0}; int dy[]={0,1,0,-1};
        string nm[]={"down","right","up","left"};
        
        while(!q.empty()){
            auto c = q.front(); q.pop_front();
            int cx=c.first.first; int cy=c.first.second;
            if(cx==tx && cy==ty) return c.second;
            
            for(int i=0;i<4;i++){
                int nx=cx+dx[i]; int ny=cy+dy[i];
                if(nx>=0 && nx<29 && ny>=0 && ny<29){
                    string v = mapa[nx][ny];
                    if((v=="f"||v=="t"||v=="r") && vs.count({nx,ny})==0){
                        vs.insert({nx,ny});
                        vector<string> nv=c.second; nv.push_back(nm[i]);
                        q.push_back({{nx,ny}, nv});
                    }
                }
            }
        }
        return {};
    }

    void main_loop() {
        while(!cl_move->wait_for_service(1s));
        while(!cl_reset->wait_for_service(1s));
        
        auto req = make_shared<cg_interfaces::srv::Reset::Request>();
        req->is_random = true;
        auto fut = cl_reset->async_send_request(req);
        spin_until_future_complete(this->shared_from_this(), fut);
        cout << "Iniciando map: " << fut.get()->loaded_map_name << endl;

        while(!dados_sensor) { spin_some(this->shared_from_this()); this_thread::sleep_for(100ms); }

        vector<pair<int,int>> pilha;
        set<pair<int,int>> visitado;
        
        visitado.insert({x,y});
        pilha.push_back({x,y});

        int ax=-1, ay=-1;
        bool ganhou=false;

        while(!pilha.empty()){
            dados_sensor=nullptr;
            while(!dados_sensor){ spin_some(this->shared_from_this()); this_thread::sleep_for(50ms); }
            sensor_to_map();

            if(mapa[x][y]=="t") { ax=x; ay=y; ganhou=true; break; }

            // Checa vizinhos (Prioridade original do Python: Down, Right, Up, Left)
            if(x+1<29 && mapa[x+1][y]=="t") { andar("down"); ax=x; ay=y; ganhou=true; break; }
            if(y+1<29 && mapa[x][y+1]=="t") { andar("right"); ax=x; ay=y; ganhou=true; break; }
            if(x-1>=0 && mapa[x-1][y]=="t") { andar("up"); ax=x; ay=y; ganhou=true; break; }
            if(y-1>=0 && mapa[x][y-1]=="t") { andar("left"); ax=x; ay=y; ganhou=true; break; }

            bool moveu = false;
            
            // Tenta mover para nao visitados (Ordem Fixa)
            // Down
            if(!moveu && x+1<29) {
                string c = mapa[x+1][y];
                if((c=="f"||c=="t") && visitado.count({x+1, y})==0) {
                    if(andar("down")) {
                        visitado.insert({x, y}); // ja esta na nova pos
                        pilha.push_back({x-1, y}); // guarda a velha na pilha
                        moveu=true;
                    }
                }
            }
            // Right
            if(!moveu && y+1<29) {
                string c = mapa[x][y+1];
                if((c=="f"||c=="t") && visitado.count({x, y+1})==0) {
                    if(andar("right")) {
                        visitado.insert({x, y});
                        pilha.push_back({x, y-1});
                        moveu=true;
                    }
                }
            }
            // Up
            if(!moveu && x-1>=0) {
                string c = mapa[x-1][y];
                if((c=="f"||c=="t") && visitado.count({x-1, y})==0) {
                    if(andar("up")) {
                        visitado.insert({x, y});
                        pilha.push_back({x+1, y});
                        moveu=true;
                    }
                }
            }
            // Left
            if(!moveu && y-1>=0) {
                string c = mapa[x][y-1];
                if((c=="f"||c=="t") && visitado.count({x, y-1})==0) {
                    if(andar("left")) {
                        visitado.insert({x, y});
                        pilha.push_back({x, y+1});
                        moveu=true;
                    }
                }
            }

            if(moveu) {
                this_thread::sleep_for(50ms);
            } else {
                // CORRECAO APLICADA AQUI TAMBEM
                if(!pilha.empty()){
                    pair<int,int> ant = pilha.back();
                    pilha.pop_back();
                    int dx = ant.first - x;
                    int dy = ant.second - y;
                    
                    // Logica invertida conforme sua correcao
                    if(dx == -1) andar("down");
                    else if(dx == 1) andar("up");
                    else if(dy == 1) andar("right");
                    else if(dy == -1) andar("left");
                    
                    this_thread::sleep_for(50ms);
                } else break;
            }
        }

        if(ganhou) {
            cout << "ALVO ENCONTRADO!" << endl;
            auto rota = verify(ax, ay);
            if(rota.size()>0) cout << "Rota valida: " << rota.size() << endl;
            else cout << "Erro rota" << endl;
        } else {
            cout << "Falha." << endl;
        }
    }
};

int main(int argc, char **argv) {
    init(argc, argv);
    auto node = make_shared<MazeSemAlvoPt2>();
    node->main_loop();
    shutdown();
    return 0;
}