class Constraint:
    def __init__(self, x1, x2, y1, y2):
        self.left = min(x1, x2)
        self.right = max(x1, x2)
        self.down = min(y1, y2)
        self.up = max(y1, y2)
