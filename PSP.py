from BMP import BMP
from BSP import BSP


class PSP:
    """
    pricing subproblem
    """

    def __init__(self, net):
        self.BMP = BMP(net)
        self.BSP = BSP()

    def solve(self, net, duals):
        self.BMP.update_objective(net, duals)
        _, x_vals = self.BMP.solve()
        dasd = 0
