class Vertex:
    def __init__(self):
        self.edge2path = None
        self.distance = -1
        self.relatedEdges = []
        self.label = ''

    def nextVertex(self):
        if edge2path is None:
            return None
        else:
            return edge2path.head

    def __str__(self):
        return self.label
    
    def __eq__(self, other):
        return

class Edge:

