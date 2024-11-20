#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp" 

#include <chrono>
#include <set>
#include <stack>
#include <string>
#include <iostream>
using namespace std::chrono_literals;
using namespace std;


//ajuda do gpt
struct Position {
    signed char x, y;

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const Position& other) const {
        return x == other.x ? y < other.y : x < other.x;
    }
};

std::shared_ptr<cg_interfaces::srv::MoveCmd::Response> move(std::shared_ptr<rclcpp::Node> node,
                                                            rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client,
                                                            const string& robot_direction) {
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = robot_direction;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return nullptr;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        return result.get();
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call move service");
        return nullptr;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("maze_client");
    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("move_command");

    //dica gpt
    stack<Position> path;
    set<Position> visited;

    // Primeira movimentação
    auto response = move(node, client, "right");
    if (!response) {
        RCLCPP_ERROR(node->get_logger(), "Failed initial move");
        return 1;
    }

    Position robot_position = {response->robot_pos[0], response->robot_pos[1]};
    Position target_position = {response->target_pos[0], response->target_pos[1]};
    path.push(robot_position);
    visited.insert(robot_position);

    while (!(target_position == robot_position)) {
        Position current = path.top();

        if (robot_position == target_position) {
            cout << "Destino alcançado!" << endl;
            rclcpp::shutdown();
            return 0;
        }

        string next_direction = "";
        Position next_position = current;
        bool moved = false;

        if (response->down == "t"){
            next_position = {current.x, current.y - 1};
            next_direction = "down";
            moved = true;
        } else if (response->right == "t"){
            next_position = {current.x + 1, current.y};
            next_direction = "right";
            moved = true;
        } else if (response->up == "t"){
            next_position = {current.x, current.y + 1};
            next_direction = "up";
            moved = true;
        } else if (response->left == "t"){
            next_position = {current.x - 1, current.y};
            next_direction = "left";
            moved = true;
        } else if (response->down == "f" && visited.find({current.x, current.y - 1}) == visited.end()) {
            next_position = {current.x, current.y - 1};
            next_direction = "down";
            moved = true;
        } else if (response->right == "f" && visited.find({current.x + 1, current.y}) == visited.end()) {
            next_position = {current.x + 1, current.y};
            next_direction = "right";
            moved = true;
        } else if (response->up == "f" && visited.find({current.x, current.y + 1}) == visited.end()) {
            next_position = {current.x, current.y + 1};
            next_direction = "up";
            moved = true;
        } else if (response->left == "f" && visited.find({current.x - 1, current.y}) == visited.end()) {
            next_position = {current.x - 1, current.y};
            next_direction = "left";
            moved = true;
        } else if (response->down == "f" && response->up=="f" && response->right=="f" && response->left=="f") {
            auto diff_x = target_position.x - current.x;
            auto diff_y = target_position.y - current.y;
            if (abs(diff_x) > abs(diff_y)) {
                next_position = {current.x + (diff_x > 0 ? 1 : -1), current.y};
                next_direction = diff_x > 0 ? "right" : "left";
            } else {
                next_position = {current.x, current.y + (diff_y > 0 ? 1 : -1)};
                next_direction = diff_y > 0 ? "up" : "down";
            }
        }

        if (moved) {
            response = move(node, client, next_direction);
            if (!response) {
                RCLCPP_ERROR(node->get_logger(), "Move failed");
                return 1;
            }
            path.push(next_position);
            visited.insert(next_position);
            robot_position = {response->robot_pos[0], response->robot_pos[1]};
            std::this_thread::sleep_for(100ms);
        } else {
            // Retrocede se não há movimentos válidos (ajuda do gpt)
            path.pop();
            if (!path.empty()) {
                Position previous = path.top();
                string back_direction = previous.x > current.x ? "left" :
                                        previous.x < current.x ? "right" :
                                        previous.y > current.y ? "down" : "up";
                move(node, client, back_direction);
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}

//-------------------------- código antigo (burro) --------------------------
// #include "rclcpp/rclcpp.hpp"
// #include "cg_interfaces/srv/move_cmd.hpp" 

// #include <chrono>
// #include <cmath>
// #include <cstdlib>
// #include <memory>
// #include <iostream>
// #include <string>
// #include <any>
// #include <set>
// #include <stack>
// using namespace std::chrono_literals;
// using namespace std;

// struct Position {
//     signed char x, y;
//     bool operator<(const Position& next_pos) const {
//         return x == next_pos.x && y == next_pos.y;
//     }
//     bool operator==(const Position& next_pos) const {
//         return x == next_pos.x && y == next_pos.y;
//     }
// };

// // std::shared_future<std::shared_ptr<cg_interfaces::srv::MoveCmd::Response>> 

// std::shared_ptr<cg_interfaces::srv::MoveCmd::Response> move(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client, string robot_direction){
//   // cria a estrutura do request
//   auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
//   std::string direction = robot_direction;
//   request->direction = direction;
//   // if (direction == "left" || direction == "right" || direction == "up" || direction == "down"){
//   //   request->direction = direction;
//   // } else {
//   //   RCLCPP_ERROR(node->get_logger(), "Direção inválida. Use 'left', 'right', 'up' ou 'down'");
//   //   return 1;
//   // }

//   while (!client->wait_for_service(1s)) {
//     if (!rclcpp::ok()) {
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   auto result = client->async_send_request(request);
//   // Wait for the result.
//   if (rclcpp::spin_until_future_complete(node, result) ==
//     rclcpp::FutureReturnCode::SUCCESS)
//   {
//     auto response = result.get();
//     RCLCPP_INFO(node->get_logger(), "Success: %s", response->success ? "True" : "False");
//     RCLCPP_INFO(node->get_logger(), "Robot Position: [%d, %d]", response->robot_pos[0], response->robot_pos[1]);
//     RCLCPP_INFO(node->get_logger(), "Target Position: [%d, %d]", response->target_pos[0], response->target_pos[1]);
//     RCLCPP_INFO(node->get_logger(), "Directions:");
//     RCLCPP_INFO(node->get_logger(), "Left: %s", response->left.c_str());
//     RCLCPP_INFO(node->get_logger(), "Down: %s", response->down.c_str());
//     RCLCPP_INFO(node->get_logger(), "Up: %s", response->up.c_str());
//     RCLCPP_INFO(node->get_logger(), "Right: %s", response->right.c_str());
//   return response;
//   } else {
//     RCLCPP_ERROR(node->get_logger(), "Falha ao chamar o serviço de mover o robô");
//     return nullptr;
//   }
// }

// int main(int argc, char **argv){
//   rclcpp::init(argc, argv);

//   //cria o nó do client   
//   std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("maze_client");

//   //cria o client do serviço
//   rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client = node->create_client<cg_interfaces::srv::MoveCmd>("move_command");

//   stack<Position> path;
//   set<Position> visited;

//   auto first_move = move(node,client,"right");
//   Position robot_position = {first_move->robot_pos[0], first_move->robot_pos[1]};
//   Position target_position = {first_move->target_pos[0], first_move->target_pos[1]};
//   auto up = first_move->up;
//   auto down = first_move->down;
//   auto left = first_move->left;
//   auto right = first_move->right;
//   auto next_move = first_move;
//   // auto manhattan_distance = abs(first_move->target_pos[0] - first_move->robot_pos[0]) + abs(first_move->target_pos[1] - first_move->robot_pos[1]);
  
//   path.push(robot_position); 
//   visited.insert(robot_position);
//   //testando busca gulosa com backtracking
//   while (!path.empty()) {
//         Position atual = path.top(); // Posição atual

//         // Verifica se chegou ao destino
//         if (robot_position.x == target_position.x && robot_position.y == target_position.y) {
//             cout << "Destino alcançado!" << endl;
//             break; }

//         Position melhorMovimento = atual;
//         int shortestPath = abs(first_move->target_pos[0] - first_move->robot_pos[0]) + abs(first_move->target_pos[1] - first_move->robot_pos[1]);
//         bool finished = false;

//         // Verifica os movimentos possíveis
//         if (down == "f") {
//             Position next = {atual.x, atual.y - 1};
//             if (visited.find(next) == visited.end()) {
//                 int dist = abs(first_move->target_pos[0] - next.x) + abs(first_move->target_pos[1] - next.y);
//                 if (dist < shortestPath) {
//                     melhorMovimento = next;
//                     shortestPath = dist;
//                     finished = true;
//                 }
//             }
//         }
//         if (right == "f") {
//             Position next = {atual.x + 1, atual.y};
//             if (visited.find(next) == visited.end()) {
//                 int dist = abs(first_move->target_pos[0] - next.x) + abs(first_move->target_pos[1] - next.y);
//                 if (dist < shortestPath) {
//                     melhorMovimento = next;
//                     shortestPath = dist;
//                     finished = true;
//                 }
//             }
//         }
//         if (left == "f") {
//             Position next = {atual.x - 1, atual.y};
//             if (visited.find(next) == visited.end()) {
//                 int dist = abs(first_move->target_pos[0] - next.x) + abs(first_move->target_pos[1] - next.y);
//                 if (dist < shortestPath) {
//                     melhorMovimento = next;
//                     shortestPath = dist;
//                     finished = true;
//                 }
//             }
//         }
//         if (up == "f") {
//             Position next = {atual.x, atual.y + 1};
//             if (visited.find(next) == visited.end()) {
//                 int dist = abs(first_move->target_pos[0] - next.x) + abs(first_move->target_pos[1] - next.y);
//                 if (dist < shortestPath) {
//                     melhorMovimento = next;
//                     shortestPath = dist;
//                     finished = true;
//                 }
//             }
//         }

//         if (finished) {
//             // Faz o movimento e registra no path
//             path.push(melhorMovimento);
//             visited.insert(melhorMovimento);
//             move (node, client, melhorMovimento.x > atual.x ? "right" : melhorMovimento.x < atual.x ? "left" : melhorMovimento.y > atual.y ? "up" : "down");
//             cout << "Movendo para: (" << melhorMovimento.x << ", " << melhorMovimento.y << ")" << endl;
//         } else {
//             // Retrocede se não há movimentos válidos
//             move (node, client, melhorMovimento.x < atual.x ? "right" : melhorMovimento.x > atual.x ? "left" : melhorMovimento.y < atual.y ? "up" : "down");
//             cout << "Retrocedendo de: (" << atual.x << ", " << atual.y << ")" << endl;
//             path.pop();
//         }
//     }


//   // salvar pontos que o robô já passou
//   // fazer rota acompanhando a parede
//   //separar diferentes algoritmos em funções pra poder testar vários dps

    
//     // while (robot_position != first_move->target_pos){
//     //   if (up == t){
//     //     next_move = move(node,client,"up");
//     //     std::this_thread::sleep_for(500ms);
//     //     moves++;
//     //   } else if (right == t){
//     //     next_move = move(node,client,"right");
//     //     std::this_thread::sleep_for(500ms);
//     //     moves++;
//     //   } else if (down == t){
//     //     next_move = move(node,client,"down");
//     //     std::this_thread::sleep_for(500ms);
//     //     moves++;
//     //   } else if (left == t){
//     //     next_move = move(node,client,"left");
//     //     std::this_thread::sleep_for(500ms);
//     //     moves++;
//     //   } else if (){

//     //   } 
      
//     //   else {
//     //     RCLCPP_INFO(node->get_logger(), "Não há caminho disponível");
//     //     break;
//     //   }

//       // if (x_diff > 0 && right == "f"){
//       //   next_move = move(node,client,"right");
//       //   std::this_thread::sleep_for(500ms);
//       //   moves++;
//       // } else if (x_diff < 0  && left == "f"){
//       //   next_move = move(node,client,"left");
//       //   moves++;
//       //   std::this_thread::sleep_for(500ms);

//       // } else if (y_diff > 0 && up == "f"){ 
//       //   next_move = move(node,client,"up");
//       //   moves++;
//       //   std::this_thread::sleep_for(500ms);
//  //   return 0;
//     //   }

//     // up = next_move->up;
//     // down = next_move->down;
//     // left = next_move->left;
//     // right = next_move->right;
//     // robot_position = next_move->robot_pos;
//     // }    
//       // } else if (y_diff < 0 && down == "f"){
//       //   next_move = move(node,client,"down");
//       //   moves++;
//       //   std::this_thread::sleep_for(500ms);

//       // }
//       // else if 
//       // (getchar() == 'q') {
//       //   RCLCPP_INFO(node->get_logger(), "Shutdown requested. Exiting.");
//       //   rclcpp::shutdown();
//     //   //   return 0;
//     //   }

//     // up = next_move->up;
//     // down = next_move->down;
//     // left = next_move->left;
//     // right = next_move->right;
//     // robot_position = next_move->robot_pos;
//     // }      
//   rclcpp::shutdown();
//   return 0;
// }