class Edge:
    def __init__(self, **kargs):
        self.fromNode = ''
        self.toNode = ''
        self.weight = 0
        self.availableLines = set([])
        
        for key, value in kargs.items():
            if key == 'fromNode':
                self.fromNode = value
            elif key == 'toNode':
                self.toNode = value
            elif key == 'weight':
                self.weight = float(value)
            elif key == 'availableLines':
                self.availableLines = value
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
        self.neighborsAvailableLines = {}
    
# replace weight with real Node object which is finded in Graph Edges
    def addEdge(self, toNodeLabel, weight, availableLine=None):
        self.neighbors[toNodeLabel] = float(weight)
        if not(availableLine is None):
            try:
                self.neighborsAvailableLines[toNodeLabel]
            except KeyError as ke:
                self.neighborsAvailableLines[toNodeLabel] = set([])
            finally:
                self.neighborsAvailableLines[toNodeLabel].add(availableLine)
            
    
    def removeEdge(self, label):
        if label in self.neighbors.keys():
            weight = self.neighbors[label]
            del self.neighbors[label]
            return weight

        else:
            return -1

    def getAdjacencyList(self):
        return self.neighbors.keys()
        
    def getAvailableLineList(self):
        fullLineList = []
        for key, lineList in self.neighborsAvailableLines.items():
            fullLineList += lineList
        
        return fullLineList
    
    def getAvailableLineSet(self):
        return set(self.getAvailableLineList())

    def getEdges(self):
        edges = []
        for s in self.neighbors.keys():
            e = Edge()
            e.fromNode = self.label
            e.toNode = s
            e.weight = self.neighbors[s]
            e.availableLines = self.neighborsAvailableLines[s]
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
            stringList.append(":")
            stringList.append(str(self.neighbors[adjNode]))
            stringList.append("( ")
            for lineNo in self.neighborsAvailableLines[neighborLabel]:
                stringList.append(lineNo)
                stringList.append(",")
            stringList.append(" )")
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
    
    def getDataFromBusFile(self, fileName):
        f = open(fileName, 'r')
        prev = None
        while True:
            line = f.readline()
            if not line: break
            if prev is None:
                prev = line.split(",")
                continue
            else:
                current = line.split(",")
                lineNo = prev[0];
                if prev[0] != current[0]:
                    prev = current
                    continue
                self.addEdge(fromNode=prev[2],toNode=current[2],weight=float(current[6]),lineNo=prev[0]);
                prev = current
                
    def addEdge(self, **kargs):
        if len(kargs) == 4:
            fromNode = kargs['fromNode']
            toNode = kargs['toNode']
            weight = kargs['weight']
            lineNo = kargs['lineNo']
            
            if fromNode not in self.nodes.keys():
                self.addNode(fromNode)
            if toNode not in self.nodes.keys():
                self.addNode(toNode)
            self.nodes[fromNode].addEdge(toNode,weight,availableLine=lineNo)
            
        for key, value in kargs.items():
            if key == 'edge':
                self.addEdge(fromNode=value.fromNode, toNode=value.toNode, weight=value.weight)
                self.addAvailableLines(value.fromNode, value.toNode, value.availableLines)
            if key == 'edges':
                for e in value:
                    self.addEdge(edge=e)
    
    def addAvailableLines(self, fromNode, toNode, availableLines):
        if availableLines is None:
            availableLines = []
        self.nodes[fromNode].neighborsAvailableLines[toNode] = availableLines
                    
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
                newNodes[adj].neighborsAvailableLines[key] = self.nodes[key].neighborsAvailableLines[adj]
        
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