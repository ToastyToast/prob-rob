

class BaseGraph:
    def __init__(self):
        pass

    def get_neighbours(self, node_id):
        raise NotImplementedError

    def get_cost(self, from_id, to_id):
        return 1

    def add_weighted(self, node_id, weight):
        return None

    def clear(self):
        pass


class Grid2D(BaseGraph):
    def __init__(self, width, height):
        super().__init__()

        self.width = width
        self.height = height

        self.blocked = {}

    def add_node(self, node_id, node_type):
        if not node_type:
            self.blocked[node_id] = 1

    def toggle_node(self, node_id):
        if node_id not in self.blocked:
            self.add_node(node_id, 0)
        else:
            del self.blocked[node_id]

    def get_neighbours(self, node_id):
        x, y = node_id
        neighbours = [
            (x-1, y),
            (x, y+1),
            (x+1, y),
            (x, y-1)
        ]
        if (x + y) % 2 == 0:
            neighbours.reverse()  # For diagonal paths
        neighbours = filter(self.is_valid, neighbours)
        neighbours = filter(self.is_passable, neighbours)
        return list(neighbours)

    def is_passable(self, node_id):
        return node_id not in self.blocked

    def is_valid(self, node_id):
        x, y = node_id
        return (0 <= x < self.width) and (0 <= y < self.height)

    def clear(self):
        self.blocked.clear()


class WeightedGrid2D(Grid2D):
    def __init__(self, width, height):
        super().__init__(width, height)

        self.weights = {}

    def add_node(self, node_id, node_type):
        if node_id not in self.weights:
            super().add_node(node_id, node_type)

    def add_weighted(self, node_id, weight):
        if self.is_passable(node_id):
            if node_id in self.weights:
                del self.weights[node_id]
            else:
                self.weights[node_id] = weight

    def get_neighbours(self, node_id):
        return super().get_neighbours(node_id)

    def get_cost(self, from_id, to_id):
        return self.weights.get(to_id, 1)

    def clear(self):
        super().clear()
        self.weights.clear()


class JPSGrid2D(Grid2D):
    def __init__(self, width, height):
        super().__init__(width, height)

    def get_neighbours(self, node_id):
        x, y = node_id
        neighbours = [
            (x-1, y-1),
            (x-1, y),
            (x-1, y+1),
            (x, y+1),
            (x+1, y+1),
            (x+1, y),
            (x+1, y-1),
            (x, y-1)
        ]
        if (x + y) % 2 == 0:
            neighbours.reverse()
        neighbours = filter(self.is_valid, neighbours)
        # neighbours = filter(self.is_passable, neighbours)
        return list(neighbours)

    def get_cost(self, from_id, to_id):
        return 1