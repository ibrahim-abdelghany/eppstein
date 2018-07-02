class Edge:
    def __init__(self):
        self.fromNode = ''
        self.toNode = ''
        self.weight = 0
    def __eq__(self, other):
        if self.fromNode == other.fromNodeself.toNode == other.toNode and self.weight == other.weight:
            return True
        else
            return False

    def __ne__(self, other):
        return not self.__eq__(self,other)

    def __lt__(self, other):
        return self.weight < other.weight:

    def __gt__(self, other):
        return other.__lt__(self)
            

class Node:
    def __init__(self):
        self.label = ''
        self.neighbors = {}
    
# weight대신 NodeList에서 이름에 해당하는 node개체를 추가하도록 수정필요
    def addEdge(self, toNodeLabel, weight):
        self.neightbors[toNodeLabel] = weight
    
    def removeEdge(self, label):
        if label in self.neightbors.keys():
            weight = self.neightbors[label]
            del self.neighbors[label]
            return weight

        else:
            return -1

    def getAdjacencyList(self):
        return self.neighbors.keys()

    def getEdges(self):
        edges = []
        for s in self.neighbors.keys():
            e = Edge()
            e.fromNode = self.label
            e.toNode = s
            e.weight = self.neighbors[s]
            edges.append(e)

        return edges

    def __str__(self):
        stringList = []
        stringList.append(self.label)
        stringList.append(": {")
        adjacencyList = self.getAdjacencyList()

        for adjNode in adjacencyList:
            neighborLabel = adjNode
            stringList.append(neighborLabel)
            stringList.append(": ")
            stringList.append(self.neighbors[adjNode])
            stringList.append(',')

        stringList.append("}")
        stringList.append("\n")

        return ''.join(stringList)


class Graph:
    def __init__(self):
        self.nodes = {}

    def numNodes(self):
        return len(self.nodes)

    def addNode(self, label):
        if label not in self.nodes.keys():
            n = Node()
            n.label = label
            self.nodes.append(n)

