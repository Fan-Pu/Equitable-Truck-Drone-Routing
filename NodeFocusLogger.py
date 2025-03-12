from pyscipopt import Eventhdlr, SCIP_EVENTTYPE


class NodeFocusLogger(Eventhdlr):
    def __init__(self, model):
        self.model = model  # Store model reference

    def eventInit(self):
        """Register the NODEFOCUS event."""
        self.model.catchEvent(SCIP_EVENTTYPE.NODEFOCUS, self)

    def eventExec(self, event):
        """Triggered when a node is focused (LP relaxation solved)."""
        node = self.model.getCurrentNode()
        stats = self.model.getStatistics()

        print(f"Node {node.getNumber()} | LP iter {stats['lp_iterations']} | Dual {self.model.getDualbound()}")

    def eventExit(self):
        """Unregister event on exit."""
        self.model.dropEvent(SCIP_EVENTTYPE.NODEFOCUS, self)
