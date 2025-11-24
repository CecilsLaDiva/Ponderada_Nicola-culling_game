import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap, Reset
from collections import deque
import time

class MazeResolver(Node):
    def __init__(self):
        super().__init__('maze_resolver')
        
        # Clientes pros servicos dos comandos
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.map_client = self.create_client(GetMap, '/get_map')
        self.reset_client = self.create_client(Reset, '/reset')

        # Esperar serviços (delay)
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando /move_command...')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando /get_map...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando /reset...')

    def reset_game(self):
        """Reinicia o jogo no mapa atual"""
        request = Reset.Request()
        request.is_random = True  
        request.map_name = ''      
        
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_map_data(self):
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_robot(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def bfs(self, grid, start, target, rows, cols):
        queue = deque([(start, [])])
        visited = set([start])
        nodes_explored = 0

        # Lista que prioriza o meio
        directions = [
            (1, 0, 'down'),
            (0, 1, 'right'),
            (-1, 0, 'up'),
            (0, -1, 'left')
        ]

        while queue:
            (curr_r, curr_c), path = queue.popleft()
            nodes_explored += 1

            if (curr_r, curr_c) == target:
                self.get_logger().info(f'--- RELATÓRIO ---')
                self.get_logger().info(f'Passos: {len(path)}')
                self.get_logger().info(f'Esforço: {nodes_explored}')
                return path

            for dr, dc, move_name in directions:
                next_r, next_c = curr_r + dr, curr_c + dc
                if 0 <= next_r < rows and 0 <= next_c < cols:
                    if grid[next_r][next_c] != 'b' and (next_r, next_c) not in visited:
                        visited.add((next_r, next_c))
                        queue.append(((next_r, next_c), path + [move_name]))
        return None

    def run(self):
    
        self.get_logger().info('Resetando o labirinto...')
        self.reset_game()
        time.sleep(0.5) 

        
        self.get_logger().info('Lendo o mapa...')
        map_res = self.get_map_data()
        
        rows, cols = map_res.occupancy_grid_shape
        flat_grid = map_res.occupancy_grid_flattened
        
        grid = []
        start_pos = None
        target_pos = None

        
        for r in range(rows):
            row_data = []
            for c in range(cols):
                val = flat_grid[r * cols + c]
                row_data.append(val)
                if val == 'r':
                    start_pos = (r, c)
                elif val == 't':
                    target_pos = (r, c)
            grid.append(row_data)

        
        if start_pos and target_pos:
            path = self.bfs(grid, start_pos, target_pos, rows, cols)
            if path:
                self.get_logger().info('Executando movimentos...')
                for step in path:
                    self.move_robot(step)
                    time.sleep(0.1)
            else:
                self.get_logger().error('Caminho não encontrado!')

def main(args=None):
    rclpy.init(args=args)
    resolver = MazeResolver()
    resolver.run()
    resolver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()