#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <stack>
#include <algorithm>
#include <iostream>
#include <thread>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"

using namespace std::chrono_literals;

// Cores para o terminal (ANSI Escape Codes)
const std::string RESET = "\033[0m";
const std::string RED = "\033[31m";     // Alvo
const std::string BLUE = "\033[34m";    // Robô
const std::string GREEN = "\033[32m";   // Caminho
const std::string WHITE = "\033[37m";   // Parede
const std::string GRAY = "\033[90m";    // Desconhecido

struct Point {
    int x, y;
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Point& other) const { return !(*this == other); }
    bool operator<(const Point& other) const { return std::tie(x, y) < std::tie(other.x, other.y); }
};

class MazeExplorer : public rclcpp::Node {
public:
    MazeExplorer() : Node("maze_explorer") {
        client_move_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", qos_profile, std::bind(&MazeExplorer::sensor_callback, this, std::placeholders::_1));

        grid_size_ = 100;
        internal_map_.resize(grid_size_, std::vector<std::string>(grid_size_, "?"));
        
        current_pos_ = {50, 50}; 
        start_pos_ = current_pos_;
        internal_map_[current_pos_.y][current_pos_.x] = "f"; 
        
        path_stack_.push(current_pos_);
        visited_[current_pos_] = true;
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr subscription_;
    
    int grid_size_;
    std::vector<std::vector<std::string>> internal_map_;
    Point current_pos_;
    Point start_pos_;
    Point target_pos_ = {-1, -1};
    
    std::stack<Point> path_stack_;
    std::map<Point, bool> visited_;
    bool finished_exploring_ = false;

    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        if (finished_exploring_) return; 

        update_map_from_sensors(msg);
        check_and_record_target(msg);
        
        // --- CHAMA A FUNÇÃO DE DESENHO ---
        display_map(); 
        // ---------------------------------

        std::string move_cmd = "";
        Point next_pos = current_pos_;

        // Lógica DFS
        if (can_move_to(current_pos_.x, current_pos_.y - 1)) { 
            move_cmd = "up"; next_pos = {current_pos_.x, current_pos_.y - 1};
        } else if (can_move_to(current_pos_.x, current_pos_.y + 1)) { 
            move_cmd = "down"; next_pos = {current_pos_.x, current_pos_.y + 1};
        } else if (can_move_to(current_pos_.x - 1, current_pos_.y)) { 
            move_cmd = "left"; next_pos = {current_pos_.x - 1, current_pos_.y};
        } else if (can_move_to(current_pos_.x + 1, current_pos_.y)) { 
            move_cmd = "right"; next_pos = {current_pos_.x + 1, current_pos_.y};
        }

        if (move_cmd != "") {
            move_robot(move_cmd);
            current_pos_ = next_pos;
            visited_[current_pos_] = true;
            path_stack_.push(current_pos_); 
        } else {
            // Backtracking
            if (path_stack_.size() > 1) {
                path_stack_.pop();
                Point prev = path_stack_.top();
                
                if (prev.x > current_pos_.x) move_cmd = "right";
                else if (prev.x < current_pos_.x) move_cmd = "left";
                else if (prev.y > current_pos_.y) move_cmd = "down";
                else if (prev.y < current_pos_.y) move_cmd = "up";
                
                move_robot(move_cmd);
                current_pos_ = prev;
            } else {
                finished_exploring_ = true;
                std::cout << "\n\nEXPLORAÇÃO CONCLUÍDA!\n";
                if (target_pos_.x != -1) solve_final_path();
                else RCLCPP_ERROR(this->get_logger(), "Alvo não encontrado.");
            }
        }
        std::this_thread::sleep_for(100ms);
    }

    // --- NOVA FUNÇÃO VISUAL ---
    void display_map() {
        // Limpa a tela (Código ANSI)
        std::cout << "\033[2J\033[1;1H"; 
        
        std::cout << "--- MAPA EM TEMPO REAL ---\n";
        std::cout << "Posição Atual: (" << current_pos_.x << ", " << current_pos_.y << ")\n";
        if (target_pos_.x != -1) std::cout << RED << "ALVO DETECTADO EM: (" << target_pos_.x << ", " << target_pos_.y << ")" << RESET << "\n";
        else std::cout << "Procurando alvo...\n";

        // Calcula limites para imprimir apenas a área explorada (Zoom dinâmico)
        int min_x = grid_size_, max_x = 0, min_y = grid_size_, max_y = 0;
        
        for(int y=0; y<grid_size_; y++) {
            for(int x=0; x<grid_size_; x++) {
                if(internal_map_[y][x] != "?") {
                    if(x < min_x) min_x = x;
                    if(x > max_x) max_x = x;
                    if(y < min_y) min_y = y;
                    if(y > max_y) max_y = y;
                }
            }
        }
        
        // Margem de segurança visual
        min_x = std::max(0, min_x - 2);
        max_x = std::min(grid_size_-1, max_x + 2);
        min_y = std::max(0, min_y - 2);
        max_y = std::min(grid_size_-1, max_y + 2);

        // Desenha o Grid
        for (int y = min_y; y <= max_y; ++y) {
            for (int x = min_x; x <= max_x; ++x) {
                if (x == current_pos_.x && y == current_pos_.y) {
                    std::cout << BLUE << "R " << RESET; // Robô
                } else if (x == target_pos_.x && y == target_pos_.y) {
                    std::cout << RED << "X " << RESET; // Alvo
                } else {
                    std::string cell = internal_map_[y][x];
                    if (cell == "b") std::cout << WHITE << "##" << RESET; // Parede
                    else if (cell == "f") std::cout << "  "; // Caminho livre
                    else std::cout << GRAY << ".. " << RESET; // Desconhecido
                }
            }
            std::cout << "\n";
        }
    }

    void update_map_from_sensors(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        internal_map_[current_pos_.y - 1][current_pos_.x] = msg->up;
        internal_map_[current_pos_.y + 1][current_pos_.x] = msg->down;
        internal_map_[current_pos_.y][current_pos_.x - 1] = msg->left;
        internal_map_[current_pos_.y][current_pos_.x + 1] = msg->right;
    }

    void check_and_record_target(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        if (target_pos_.x != -1) return;
        if (msg->up == "t") target_pos_ = {current_pos_.x, current_pos_.y - 1};
        else if (msg->down == "t") target_pos_ = {current_pos_.x, current_pos_.y + 1};
        else if (msg->left == "t") target_pos_ = {current_pos_.x - 1, current_pos_.y};
        else if (msg->right == "t") target_pos_ = {current_pos_.x + 1, current_pos_.y};
    }

    bool can_move_to(int x, int y) {
        if (x < 0 || y < 0 || x >= grid_size_ || y >= grid_size_) return false;
        if (visited_.count({x, y})) return false;
        std::string cell = internal_map_[y][x];
        return (cell != "b" && cell != "?" && cell != "t"); 
    }

    void move_robot(std::string direction) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = direction;
        auto future = client_move_->async_send_request(req);
    }

    void solve_final_path() {
        std::cout << "\nCalculando rota ótima no mapa descoberto...\n";
        std::queue<Point> q;
        q.push(start_pos_);
        std::map<Point, Point> came_from;
        std::vector<std::vector<bool>> bfs_visited(grid_size_, std::vector<bool>(grid_size_, false));
        bfs_visited[start_pos_.y][start_pos_.x] = true;
        
        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};
        bool found = false;

        while(!q.empty()){
            Point curr = q.front(); q.pop();
            if(curr == target_pos_) { found = true; break; }

            for(int i=0; i<4; i++){
                int nx = curr.x + dx[i];
                int ny = curr.y + dy[i];
                if(nx>=0 && ny>=0 && nx<grid_size_ && ny<grid_size_){
                    std::string cell = internal_map_[ny][nx];
                    if(!bfs_visited[ny][nx] && cell != "b" && cell != "?"){
                        bfs_visited[ny][nx] = true;
                        came_from[{nx,ny}] = curr;
                        q.push({nx,ny});
                    }
                }
            }
        }

        if(found){
            int steps = 0;
            Point curr = target_pos_;
            while(curr != start_pos_){
                curr = came_from[curr];
                steps++;
            }
            std::cout << GREEN << "SUCESSO! Menor caminho: " << steps << " passos." << RESET << "\n";
        } else {
            std::cout << RED << "ERRO: Caminho não encontrado." << RESET << "\n";
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}