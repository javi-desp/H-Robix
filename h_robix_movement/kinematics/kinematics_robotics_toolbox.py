import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from math import pi

class HEXAPOD_LEG(DHRobot):
    def __init__(self):
        deg = pi / 180

        # robot length values (metres)
        a = [0.024, 0.1, 0.15049]
        alpha = [-pi / 2, 0, 0]
        links = []

        for j in range(3):
            link = RevoluteDH(
                a=a[j], alpha=alpha[j]
            )
            links.append(link)

        super().__init__(
            links,
            name="hexapod_leg"
        )

        self.qr = np.array([0, -45, 90]) * deg
        self.qz = np.zeros(3)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    hexapod_leg = HEXAPOD_LEG()
    print(hexapod_leg)
    ang = [0, -0.51, -1.81]
    ang[2] = -ang[2]
    hexapod_leg.plot(ang)
    T = hexapod_leg.fkine(ang)
    print(T)
    sol = hexapod_leg.ikine_LM(T)  
    print(sol[0])
    input()