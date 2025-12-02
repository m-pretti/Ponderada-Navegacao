#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

using namespace std::chrono_literals;

// Estrutura para coordenadas (x = coluna, y = linha)
struct Point {
    int x;
    int y;
    
    // Operadores necessários para usar Point em sets/maps
    bool operator<(const Point& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
};

class MazeSolver : public rclcpp::Node {
public:
    MazeSolver() : Node("etapa01") {
        client_get_map_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        client_move_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    }

    void solve() {
        // 1. Obter o Mapa
        while (!client_get_map_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Aguardando serviço /get_map...");
        }

        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto result_future = client_get_map_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao chamar /get_map");
            return;
        }

        auto response = result_future.get();
        
        // 2. Converter para Matriz 2D
        // O shape vem como string no output que você mandou, precisamos converter
        // CORRETO: Atribuição direta
        int width = response->occupancy_grid_shape[0];
        int height = response->occupancy_grid_shape[1];
        auto raw_data = response->occupancy_grid_flattened;

        // Matriz 2D de Strings
        std::vector<std::vector<std::string>> grid(height, std::vector<std::string>(width));
        Point start = {0, 0}; 
        Point target = {0, 0};

        RCLCPP_INFO(this->get_logger(), "Mapeando Grid %dx%d...", width, height);

        for (int i = 0; i < height; ++i) { // i = Linha (y)
            for (int j = 0; j < width; ++j) { // j = Coluna (x)
                int index = i * width + j;
                std::string cell = raw_data[index];
                grid[i][j] = cell;

                if (cell == "r") {
                    start = {j, i};
                    RCLCPP_INFO(this->get_logger(), "Robô encontrado em (%d, %d)", j, i);
                } else if (cell == "t") {
                    target = {j, i};
                    RCLCPP_INFO(this->get_logger(), "Alvo encontrado em (%d, %d)", j, i);
                }
            }
        }

        // 3. Rodar BFS
        auto path = bfs(grid, start, target, width, height);

        // 4. Executar Movimentos
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Nenhum caminho encontrado! Verifique se o alvo está acessível.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Caminho calculado: %zu passos.", path.size());
            // Pequeno delay para dar tempo de abrir a janela visualmente se precisar
            std::this_thread::sleep_for(1s);

            for (const auto& move : path) {
                move_robot(move);
                // Delay para animação fluida (se for muito rápido o simulador pode ignorar)
                std::this_thread::sleep_for(100ms); 
            }
            RCLCPP_INFO(this->get_logger(), "Missão Cumprida!");
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_get_map_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_;

    void move_robot(std::string direction) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = direction;
        auto future = client_move_->async_send_request(req);
        // Espera síncrona pelo movimento terminar antes de enviar o próximo
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }

    // Algoritmo BFS (Busca em Largura)
    std::vector<std::string> bfs(const std::vector<std::vector<std::string>>& grid, 
                                 Point start, Point target, int width, int height) {
        
        std::queue<Point> q;
        q.push(start);
        
        std::map<Point, Point> came_from; // Rastreia o pai de cada nó
        std::map<Point, std::string> move_taken; // Rastreia qual movimento levou ao nó
        
        std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
        visited[start.y][start.x] = true;

        // Ordem de exploração: Cima, Baixo, Esquerda, Direita
        int dx[] = {0, 0, -1, 1}; 
        int dy[] = {-1, 1, 0, 0};
        std::string move_names[] = {"up", "down", "left", "right"};

        bool found = false;

        while (!q.empty()) {
            Point current = q.front();
            q.pop();

            if (current == target) {
                found = true;
                break;
            }

            for (int i = 0; i < 4; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                Point next = {nx, ny};

                // Verifica limites do mapa
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    // Verifica se não é parede ('b') e não foi visitado
                    if (!visited[ny][nx] && grid[ny][nx] != "b") {
                        visited[ny][nx] = true;
                        came_from[next] = current;
                        move_taken[next] = move_names[i];
                        q.push(next);
                    }
                }
            }
        }

        std::vector<std::string> path;
        if (!found) return path;

        // Reconstrói o caminho de trás para frente (Target -> Start)
        Point curr = target;
        while (curr != start) {
            path.push_back(move_taken[curr]);
            curr = came_from[curr];
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeSolver>();
    node->solve();
    rclcpp::shutdown();
    return 0;
}