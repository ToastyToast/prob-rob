from queue import Queue, PriorityQueue
from shortest_path.heuristics import manhattan, euclidean


class BaseAlgorithm:

    def __init__(self):
        pass

    def find_path(self, graph, start_id, end_id, heuristic=None):
        raise NotImplementedError


class BFS(BaseAlgorithm):

    def __init__(self):
        super().__init__()

    def find_path(self, graph, start_id, end_id, heuristic=None):

        flood = Queue()
        flood.put(start_id)
        visited = {}
        visited[start_id] = None

        while not flood.empty():
            current = flood.get()

            if current == end_id:
                break

            for n in graph.get_neighbours(current):
                if n not in visited:
                    flood.put(n)
                    visited[n] = current

        return visited,


class Dijkstra(BaseAlgorithm):
    
    def __init__(self):
        super().__init__()

    def find_path(self, graph, start_id, end_id, heuristic=None):
        # (priority, data)
        flood = PriorityQueue()
        flood.put((0, start_id))
        visited = {}
        visited_cost = {}
        visited[start_id] = None
        visited_cost[start_id] = 0

        while not flood.empty():
            current = flood.get()[1]

            if current == end_id:
                break

            for n in graph.get_neighbours(current):
                updated_cost = visited_cost[current] + graph.get_cost(current, n)
                if n not in visited_cost or updated_cost < visited_cost[n]:
                    visited_cost[n] = updated_cost
                    priority = updated_cost
                    flood.put((priority, n))
                    visited[n] = current

        return (visited, visited_cost)


class AStar(BaseAlgorithm):

    def __init__(self):
        super().__init__()

    def find_path(self, graph, start_id, end_id, heuristic=euclidean):
        if heuristic is None:
            raise ValueError

        # (priority, data)
        flood = PriorityQueue()
        flood.put((0, start_id))
        visited = {}
        visited_cost = {}
        visited[start_id] = None
        visited_cost[start_id] = 0

        while not flood.empty():
            current = flood.get()[1]

            if current == end_id:
                break

            for n in graph.get_neighbours(current):
                updated_cost = visited_cost[current] + graph.get_cost(current, n)
                if n not in visited_cost or updated_cost < visited_cost[n]:
                    visited_cost[n] = updated_cost
                    nx, ny = n
                    ex, ey = end_id
                    priority = updated_cost + heuristic(ex, ey, nx, ny)
                    flood.put((priority, n))
                    visited[n] = current

        return visited, visited_cost


class JumpPointSearch(BaseAlgorithm):

    def __init__(self):
        super().__init__()

    def direction(self, parent_id, curr_id):
        px, py = parent_id
        cx, cy = curr_id

        x = cx - px
        y = cy - py

        x = max(min(1, x), -1)
        y = max(min(1, y), -1)

        return x, y

    def is_diagonal(self, d):
        dx, dy = d
        return abs(dx) == abs(dy) and (dx != 0 and dy != 0)

    def prune(self, graph, d, curr_id):
        neighbours = graph.get_neighbours(curr_id)
        dx, dy = d
        cx, cy = curr_id
        pruned = []
        wall_list = []

        walls = False

        if d == (0, 0):
            return neighbours, walls

        for n in neighbours:
            if not graph.is_passable(n):
                walls = True
                wall_list.append(n)

        # diagonal
        if self.is_diagonal(d):
            pruned.append((cx + dx, cy + dy))
            pruned.append((cx, cy + dy))
            pruned.append((cx + dx, cy))
        else:  # straight
            pruned.append((cx + dx, cy + dy))

        if walls:  # forced neighbours
            for w in wall_list:
                if w not in pruned:
                    wx, wy = w
                    forced_neighbours = [  # TODO: better solution?
                        (wx + dx, wy + dy),
                        (wx, wy + dy),
                        (wx + dx, dy),
                    ]

                    for forced_neighbour in forced_neighbours:
                        if forced_neighbour not in pruned and forced_neighbour not in wall_list \
                           and graph.is_valid(forced_neighbour):
                            pruned.append(forced_neighbour)

        return pruned, walls

    def jump(self, graph, x, d, start_id, end_id):
        cx, cy = x
        dx, dy = d

        n = cx+dx, cy+dy

        if not graph.is_valid(n) or not graph.is_passable(n):
            return None
        if n == end_id:
            return n
        neighbours, has_forced = self.prune(graph, d, n)
        if has_forced:
            return n

        if self.is_diagonal(d):
            dirs = [(0, dy), (dx, 0)]
            for dr in dirs:
                if self.jump(graph, n, dr, start_id, end_id) is not None:
                    return n
        return self.jump(graph, n, d, start_id, end_id)

    def find_path(self, graph, start_id, end_id, heuristic=euclidean):
        if heuristic is None:
            raise ValueError

        # (priority, data)
        flood = PriorityQueue()
        flood.put((0, start_id))
        visited = {}
        visited_cost = {}
        visited[start_id] = None
        visited_cost[start_id] = 0

        prev = start_id

        while not flood.empty():
            current = flood.get()[1]

            if current == end_id:
                break

            #neighbours, walls = self.prune(graph, self.direction(prev, current), current)

            for n in graph.get_neighbours(current):
                updated_cost = visited_cost[current] + graph.get_cost(current, n)
                jn = self.jump(graph, current, self.direction(current, n), start_id, end_id)
                if jn is None:
                    continue
                if jn not in visited_cost or updated_cost < visited_cost[jn]:
                    visited_cost[jn] = updated_cost
                    nx, ny = jn
                    ex, ey = end_id
                    priority = updated_cost + heuristic(ex, ey, nx, ny)
                    flood.put((priority, jn))
                    visited[jn] = current

            prev = current

        return visited, {}

