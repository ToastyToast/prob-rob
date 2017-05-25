from shortest_path.heuristics import euclidean


class Pathfinder:
    def __init__(self):
        self.algorithm = None
        self.graph = None
        self.start = None
        self.finish = None
        self.visited = {}
        self.path = []
        self.heuristic = euclidean

    def set_params(self, algo, graph, start, finish, heuristic=euclidean):
        self.algorithm = algo
        self.graph = graph
        self.start = start
        self.finish = finish
        self.heuristic = heuristic

    def find_path(self):
        if self.algorithm is None and self.graph is None:
            return []

        self.visited.clear()
        self.path.clear()

        self.visited = self.algorithm.find_path(self.graph, self.start, self.finish, heuristic=self.heuristic)[0]

        if self.finish in self.visited:
            current = self.finish
            self.path = [current]
            while current != self.start:
                current = self.visited[current]
                self.path.append(current)
            self.path.reverse()

        return self.path

    def get_next_visited(self):
        pass

    def get_next_path(self):
        pass

