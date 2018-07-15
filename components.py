class Edge:
    def __init__(self, **kargs):
        self.fromNode = ''
        self.toNode = ''
        self.weight = 0
        for key, value in kargs.items():
            if key == 'fromNode':
                self.fromNode = value
            elif key == 'toNode':
                self.toNode = value
            elif key == 'weight':
                self.weight = float(value)
        # if len(args) > 0:
        #     self.weight = args[0]
        # else:
        #     self.weight = 0
    def __eq__(self, other):
        if self.fromNode == other.fromNode and self.toNode == other.toNode and self.weight == other.weight:
            return True
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        return self.weight < other.weight

    def __gt__(self, other):
        return other.__lt__(self)
        
    def __str__(self):
        return "(%s->%s : %f)" %(self.fromNode,self.toNode,self.weight)
    
    def __repr__(self):
        return self.__str__()
            

class Node:
    def __init__(self):
        self.label = ''
        self.neighbors = {}
    
# replace weight with real Node object which is finded in Graph Edges
    def addEdge(self, toNodeLabel, weight):
        self.neighbors[toNodeLabel] = float(weight)
    
    def removeEdge(self, label):
        if label in self.neighbors.keys():
            weight = self.neighbors[label]
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
            stringList.append(str(self.neighbors[adjNode]))
            stringList.append(',')

        stringList.append("}")
        stringList.append("\n")

        return ''.join(stringList)
    
    def __repr__(self):
        return self.__str__()
        
    def equals(self, other):
        if self.label == other.label:
            return True
        else:
            return False

class Graph:
    def __init__(self):
        self.nodes = {}

    def numNodes(self):
        return len(self.nodes)

    def addNode(self, label):
        if label not in self.nodes.keys():
            n = Node()
            n.label = label
            self.nodes[label] = n
    
    def getDataFromFile(self, fileName):
        f = open(fileName, 'r')
        while True:
            line = f.readline()
            if not line: break
            edgeDescription = line.split()
            if len(edgeDescription) == 3:
                self.addEdge(fromNode=edgeDescription[0],toNode=edgeDescription[1],weight=edgeDescription[2])
            
        f.close()
     
    def addEdge(self, **kargs):
        if len(kargs) == 3:
            fromNode = kargs['fromNode']
            toNode = kargs['toNode']
            weight = kargs['weight']
            
            if fromNode not in self.nodes.keys():
                self.addNode(fromNode)
            if toNode not in self.nodes.keys():
                self.addNode(toNode)
            self.nodes[fromNode].addEdge(toNode,weight)
            
        for key, value in kargs.items():
            if key == 'edge':
                self.addEdge(fromNode=value.fromNode, toNode=value.toNode, weight=value.weight)
            if key == 'edges':
                for e in value:
                    self.addEdge(edge=e)
                    
    def transpose(self):
        newNodes = {}
        
        for key, node in self.nodes.items():
            n = Node()
            n.label = key
            newNodes[key] = n
        
        for key, node in self.nodes.items():
            adjacencyList = node.getAdjacencyList()
            
            neighbors = node.neighbors
            for adj in adjacencyList:
                newNodes[adj].addEdge(key,neighbors[adj])
        
        g = Graph()
        g.nodes = newNodes
        return g
    
    def getEdgeList(self):
        edgeList = []
        
        for node in self.nodes:
            edgeList += self.nodes[node].getEdges()

        return edgeList

    def getNode(self,label):
        return self.nodes[label]