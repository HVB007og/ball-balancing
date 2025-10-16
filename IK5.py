import math

class IK:

    def __init__(self, d, e, f, g, hz):
        self.d = d
        self.e = e
        self.f = f
        self.g = g
        self.hz = hz

    def unit_normal(self, nx, ny):
        nmag = math.sqrt(nx**2 + ny**2 + 1)
        nx /= nmag
        ny /= nmag
        nz = 1 / nmag
        return nx, ny, nz

    def calculate_a(self, ny, nz):
        # Leg A calculation
        nx = 0  # Assume nx = 0 for leg A
        ny, nz = self.unit_normal(nx, ny)[1:]  # Get unit normal components
        y = self.d + (self.e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2))))
        z = self.hz + self.e * ny
        mag = math.sqrt(y**2 + z**2)
        angle = math.acos(y / mag) + math.acos((mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))
        theta_a = angle * (180 / math.pi)
        return theta_a

    def calculate_b(self, nx, ny, nz):
        # Leg B calculation
        nx, ny, nz = self.unit_normal(nx, ny)  # Normalize the normal vector
        x = (math.sqrt(3) / 2) * (self.e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - self.d)
        y = x / math.sqrt(3)
        z = self.hz - (self.e / 2) * (math.sqrt(3) * nx + ny)
        mag = math.sqrt(x**2 + y**2 + z**2)
        angle = math.acos((math.sqrt(3) * x + y) / (-2 * mag)) + math.acos((mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))
        theta_b = angle * (180 / math.pi)
        return theta_b

    def calculate_c(self, nx, ny, nz):
        # Leg C calculation
        nx, ny, nz = self.unit_normal(nx, ny)  # Normalize the normal vector
        x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1)))
        y = -x / math.sqrt(3)
        z = self.hz + (self.e / 2) * (math.sqrt(3) * nx - ny)
        mag = math.sqrt(x**2 + y**2 + z**2)
        angle = math.acos((math.sqrt(3) * x - y) / (2 * mag)) + math.acos((mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))
        theta_c = angle * (180 / math.pi)
        return theta_c
