import math


#d = 5.0
#e = 7.937
#f = 4.845
#g = 8.72
#hz = 12.5


class IK:

    def __init__(self, d, e, f, g, hz):

        self.d = d
        self.e = e
        self.f = f
        self.g = g
        self.hz = hz

    def calculate_a(self, ny, nz):
        c = ((self.e*ny)/math.sqrt(abs(math.pow(nz, 2) + math.pow(ny, 2)))) + self.hz
        b = -math.sqrt(abs(math.pow(self.e, 2) - math.pow(c - self.hz, 2)))

        I = (((b+self.d)/c)**2) + 1
        J = (((self.g**2)*(b+self.d))/(c**2)) - (b+self.d) + (2*self.d)
        K = ((c**2)/4) + ((self.g**4)/(4*(c**2))) - ((self.g**2)/2) - (self.f**2) + (self.d**2)

        Y = (-J - math.sqrt(abs((J**2) - (4*I*K))))/(2*I)
        Z = math.sqrt(abs((self.f**2) - ((Y + self.d)**2)))

        theta_a = math.atan((-Z)/(Y + self.d)) * (180/math.pi)
        return(-theta_a)

    def calculate_b(self, nx, ny, nz):
        c = ((self.e*((nx*math.sqrt(3)) + ny))/math.sqrt(abs((nz**2) + (((nx*math.sqrt(3)) + ny)**2)))) + self.hz
        a = math.sqrt(abs(3*((self.e**2) - ((c - self.hz)**2))))
        b = a/math.sqrt(3)

        I = ((((4*self.d)/(math.sqrt(3))) - (2*a) - ((2*b)/(math.sqrt(3))))**2) + ((16*(c**2))/3)
        J = (2*(((4*self.d)/(math.sqrt(3))) - (2*a) - ((2*b)/(math.sqrt(3))))*((a**2) + (b**2) - (self.d**2) + (self.f**2) + (c**2) - (self.g**2))) - ((16*self.d*(c**2))/(math.sqrt(3)))
        K = (((a**2) + (b**2) - (self.d**2) + (self.f**2) + (c**2) - (self.g**2))**2) + (4*(self.d**2)*(c**2)) - (4*(self.f**2)*(c**2))

        X = (-J + math.sqrt(abs((J**2) - (4*I*K))))/(2*I)
        Y = X/math.sqrt(3)
        Z = math.sqrt(abs(((((X*4*self.d)/(math.sqrt(3))) - (self.d**2) + (self.f**2) - ((4*(X**2))/(3))))))

        theta_b = math.atan(Z/((2*Y) - self.d)) * (180/math.pi)
        return(-theta_b)

    def calculate_c(self, nx, ny, nz):
        c = ((self.e*(ny - (math.sqrt(3)*nx)))/(math.sqrt(abs((4*(nz**2)) + ((ny - (math.sqrt(3)*nx))**2))))) + self.hz
        a = -math.sqrt(abs(3*((self.e**2) - ((self.hz-c)**2))))/2
        b = -a/math.sqrt(3)

        I = ((((4*self.d)/(math.sqrt(3))) + (2*a) + ((2*b)/(math.sqrt(3))))**2) + ((16*(c**2))/3)
        J = (2*(((4*self.d)/(math.sqrt(3))) + (2*a) + ((2*b)/(math.sqrt(3))))*((a**2) + (b**2) - (self.d**2) + (self.f**2) + (c**2) - (self.g**2))) + ((16*self.d*(c**2))/(math.sqrt(3)))
        K = (((a**2) + (b**2) - (self.d**2) + (self.f**2) + (c**2) - (self.g**2))**2) + (4*(self.d**2)*(c**2)) - (4*(self.f**2)*(c**2))

        X = (J - math.sqrt(abs((J**2) - (4*I*K))))/(2*I)
        Y = -X/math.sqrt(3)
        Z = math.sqrt(abs((self.f**2) - (self.d**2) - ((4*(X**2))/(3)) - ((X*4*self.d)/(math.sqrt(3)))))

        theta_c = math.atan((Z)/((2*Y) - self.d)) * (180/math.pi)
        return(theta_c)