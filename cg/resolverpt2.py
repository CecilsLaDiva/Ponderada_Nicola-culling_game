import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, Reset
from cg_interfaces.msg import RobotSensors
from collections import deque
import time

TARGET_POS = (14, 14)

class MazeResolverpt2(Node):
    def __init__(self):
        super().__init__('maze_resolverpt2')

        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.reset_client = self.create_client(Reset, '/reset')
        
        self.sensor_sub = self.create_subscription(
            RobotSensors, 
            '/culling_games/robot_sensors', 
            self.sensor_callback, 
            10
        )
        
        self.latest_sensors = None
        self.robot_pos = [1, 1]
        self.stack = []
        self.visited = set()
        
        self.rows, self.cols = 29, 29
        self.internal_map = [['?' for _ in range(self.cols)] for _ in range(self.rows)]
        self.internal_map[1][1] = 'f'

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando move_command...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando reset...')

    def sensor_callback(self, msg):
        self.latest_sensors = msg

    def reset_game(self):
        req = Reset.Request()
        req.is_random = True
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_robot(self, direction):
        req = MoveCmd.Request()
        req.direction = direction
        future = self.move_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.robot_pos = list(res.robot_pos)
        return res.success

    def update_map_from_sensors(self):
        if not self.latest_sensors:
            return

        r, c = self.robot_pos
        sensors = self.latest_sensors
        
        deltas = {
            'up': (-1, 0, sensors.up),
            'down': (1, 0, sensors.down),
            'left': (0, -1, sensors.left),
            'right': (0, 1, sensors.right)
        }

        for direction, (dr, dc, val) in deltas.items():
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.internal_map[nr][nc] == '?':
                    self.internal_map[nr][nc] = val

    def bfs_check(self, start, target):
        """Valida o mapa gerado encontrando um caminho nela"""
        queue = deque([(tuple(start), [])])
        visited_bfs = set([tuple(start)])
        
        # Heurística de movimento para o BFS também
        directions = [(1, 0, 'down'), (0, 1, 'right'), (-1, 0, 'up'), (0, -1, 'left')]

        while queue:
            (curr_r, curr_c), path = queue.popleft()
            if (curr_r, curr_c) == tuple(target):
                return path

            for dr, dc, move_name in directions:
                nr, nc = curr_r + dr, curr_c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    cell = self.internal_map[nr][nc]
                    # Só anda se for chão conhecido
                    if cell in ['f', 't', 'r'] and (nr, nc) not in visited_bfs:
                        visited_bfs.add((nr, nc))
                        queue.append(((nr, nc), path + [move_name]))
        return None

    def run(self):
        self.get_logger().info('Iniciando Exploração/mapeamento de área ...')
        res = self.reset_game()
        self.get_logger().info(f'Mapa Aleatório Loaded: {res.loaded_map_name}')
        
        # Espera inicial pelos sensores
        while self.latest_sensors is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.visited.add(tuple(self.robot_pos))
        self.stack.append(tuple(self.robot_pos))
        
        target_found_pos = None

        while self.stack:
            self.latest_sensors = None
            while self.latest_sensors is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            self.update_map_from_sensors()
            curr_r, curr_c = self.robot_pos

            if self.internal_map[curr_r][curr_c] == 't':
                target_found_pos = (curr_r, curr_c)
                break
                
            nearby_target = False
            for dr, dc, move in [(1,0,'down'),(0,1,'right'),(-1,0,'up'),(0,-1,'left')]:
                 nr, nc = curr_r + dr, curr_c + dc
                 if 0 <= nr < self.rows and 0 <= nc < self.cols:
                     if self.internal_map[nr][nc] == 't':
                         self.move_robot(move)
                         target_found_pos = (nr, nc)
                         nearby_target = True
                         break
            if nearby_target:
                break

            valid_moves = []
            possible_directions = [('down', 1, 0), ('right', 0, 1), ('up', -1, 0), ('left', 0, -1)]
            
            for move_name, dr, dc in possible_directions:
                nr, nc = curr_r + dr, curr_c + dc
                
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    cell_val = self.internal_map[nr][nc]
                    
                    if cell_val in ['f', 't'] and (nr, nc) not in self.visited:

                        dist_to_center = abs(nr - TARGET_POS[0]) + abs(nc - TARGET_POS[1])
                        

                        valid_moves.append((dist_to_center, move_name, dr, dc, nr, nc))
            
            valid_moves.sort(key=lambda x: x[0])

            if valid_moves:
                _, best_move, _, _, nr, nc = valid_moves[0]
                
                if self.move_robot(best_move):
                    self.visited.add((nr, nc))
                    self.stack.append((curr_r, curr_c))
                    time.sleep(0.05)
                else:
                    pass
            else:
                if len(self.stack) > 0:
                    back_r, back_c = self.stack.pop()
                    dr = back_r - curr_r
                    dc = back_c - curr_c
                    
                    back_move = ''
                    if dr == 1: back_move = 'down'
                    elif dr == -1: back_move = 'up'
                    elif dc == 1: back_move = 'right'
                    elif dc == -1: back_move = 'left'
                    
                    if back_move:
                        self.move_robot(back_move)
                        time.sleep(0.05)
                else:
                    break

        if target_found_pos:
            self.get_logger().info("--- ALVO ENCONTRADO! ---")
            self.get_logger().info("Comprovando mapa com BFS...")
            path = self.bfs_check((1,1), target_found_pos)
            if path:
                self.get_logger().info(f"SUCESSO! Mapa válido. Rota de {len(path)} passos calculada.")
            else:
                self.get_logger().error("ERRO: Falha ao calcular rota no mapa gerado.")
        else:
            self.get_logger().warn("Exploração finalizada sem encontrar o alvo.")

def main(args=None):
    rclpy.init(args=args)
    resolverpt2 = MazeResolverpt2()
    resolverpt2.run()
    resolverpt2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()