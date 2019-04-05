class Quaternion:
    """
        Class for rotation quaternions
    """

    def __init__(self, x = 0, y = 0, z = 0, w = 1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def setValues(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def norm(self):
        mag = pow(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2) + pow(self.w, 2), 0.5)

        if mag > 0:
            self.x = self.x / mag
            self.y = self.y / mag
            self.z = self.z / mag
            self.w = self.w / mag


def quatMultiply(p, q):
    w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z
    x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y
    y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x
    z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w
    
    pq = Quaternion(x, y, z, w)
    pq.norm()

    return pq