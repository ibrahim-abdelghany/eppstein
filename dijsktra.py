from heapq import heappush,heappop,heapify
from components import *

class DijsktraNode(Node):
    def __init__(self, **kwargs):
        super().__init__()
        for key, value in kwargs.items():
            if key == 'dist':
                self.dist = float(value)
            elif key == 'label':
                self.label = value
            elif key == 'depth':
                self.depth = int(value)
            elif key == 'parent':
                self.setParent(value)
                super().addEdge(value,0.0)
        else:
            self.dist = float("inf")
            self.depth = 99999
    
    def setParent(self, parent):
        self.neighbors = {}
        self.neighbors[parent] = 0.0
    
    def getParent(self):
        neighborLabels = self.neighbors.keys()
        if len(neighborLabels) != 1:
            return None
        
        for key in self.neighbors.keys():
            return key
        
    def __hash__(self):
        return hash(self.label)
    
    def __lt__(self, other):
        return self.dist < other.dist
    
    def __gt__(self,other):
        return other.__lt__(self)
    
    def __eq__(self,other):
        return self.dist == other.dist
    
    def __ne__(self,other):
        return not self.__eq__(other)
        
    def __repr__(self):
        return self.label

class Path:
    def __init__(self, **kwargs):
        for key, value in kwargs:
            if key == 'totalCost':
                self.totalCost = value
            elif key == 'edges':
                self.edges = value
                totalCost = 0
                for e in edges:
                    totalCost += e.weight
                self.totalCost = totalCost
        
        else:
            self.edges = []
            self.totalCost = 0
            
    
    def getPathNodes(self):
        nodes = []
        
        for e in self.edges:
            nodes.append(e.fromNode)
        
        lastEdge = edges[len(self.edges)-1]
        if lastEdge is not None:
            nodes.append(lastEdge.toNode)
        
        return nodes
    
    def addFirstNode(self, nodeLabel):
        firstNode = self.edges[0].fromNode
        #TODO: need to find edge in edgeList not make new
        self.edges.insert(0, Edge(fromNode=nodeLabel, toNode=firstNode, weight=0))
        
    def addFirst(self, edge):
        self.edges.insert(0, edge)
        self.totalCost += edge.weight
    
    def add(self,edge):
        self.edges.append(edge)
        self.totalCost += edge.weight
    
    def addLastNode(self,nodeLabel):
        lastNode = self.edges[len(self.edges)-1].toNode
        e = Edge()
        e.fromNode = lastNode
        e.toNode = nodeLabel
        e.weight = 0
        edges.append(e)
        
    def __str__(self):
        sb = []
        numEdges = len(self.edges)
        sb.append(str(self.totalCost))
        sb.append(": [")
        if numEdges > 0:
            for i in range(numEdges):
                sb.append(self.edges[i].fromNode)
                sb.append("-")
        
            sb.append(self.edges[numEdges-1].toNode)
        
        sb.append("]")
        
        return ''.join(sb)

class ShortestPathTree:
    def __init__(self, **kwargs):
        self.nodes = {}
        for key, value in kwargs.items():
            if key == "root":
                self.root = value
        
            
    def add(self, newNode):
        self.nodes[newNode.label] = newNode
    
    def setParentOf(self, node, parent):
        #if parent is not None and parent not in nodes.keys():
        if node not in self.nodes.keys():
            self.nodes[node] = DijsktraNode(label=node)
        self.nodes[node].setParent(parent)
    
    def getParentOf(self, node):
        if node in self.nodes.keys():
            return self.nodes[node].getParent()
        else:
            return None
        
    
def shortestPathTree(graph, sourceLabel):
    nodes = graph.nodes
    if sourceLabel not in nodes.keys():
        raise Exception("source not in nodes")

    visited = set([])
    predecessorTree = ShortestPathTree(root=sourceLabel)
    pq = []

    for nodeLabel in nodes.keys():
        node = DijsktraNode(label=nodeLabel)
        node.dist = float("inf")
        node.depth = 99999
        predecessorTree.add(node)

    sourceNode = predecessorTree.nodes[predecessorTree.root]
    sourceNode.dist = 0
    sourceNode.depth = 0
    heappush(pq, sourceNode)

    count = 0

    while len(pq) > 0:
        current = heappop(pq)
        curLabel = current.label
        visited.add(current)
        count += 1

        neighbors = nodes.get(curLabel).neighbors
        for curNeighborLabel in neighbors.keys():
            neighborNode = predecessorTree.nodes[curNeighborLabel]
            curDistance = neighborNode.dist
            newDistance = current.dist + nodes[curLabel].neighbors[curNeighborLabel]
            #print("curDis : %f, newDis : %f" % (curDistance, newDistance))
            if newDistance < curDistance:
                neighbor = predecessorTree.nodes[curNeighborLabel]
                if neighbor in pq:
                    pq.remove(neighbor)
                neighbor.depth = current.depth + 1
                neighbor.dist = newDistance
                neighbor.setParent(curLabel)
                pq.append(neighbor)
                heapify(pq)
    
    return predecessorTree
    
def shortestPath(graph, sourceLabel, targetLabel):
    nodes = graph.nodes
    predecessorTree = ShortestPathTree(root=sourceLabel)
    pq = []
    
    for nodeLabel in nodes.keys():
        node = DijsktraNode(label=nodeLabel)
        node.dist = float("inf")
        node.depth = 99999
        predecessorTree.add(node)
    
    sourceNode = predecessorTree.nodes[predecessorTree.root]
    sourceNode.dist = 0
    sourceNode.depth = 0
    heappush(pq,sourceNode)
    
    count = 0
    while len(pq) > 0:
        current = heappop(pq)
        print("this is poped up")
        print(current)
        curLabel = current.label
        
                
        count += 1
        neighbors = nodes[curLabel].neighbors
        for curNeighborLabel in neighbors.keys():
            neighborNode = predecessorTree.nodes[curNeighborLabel]
            curDistance = neighborNode.dist
            newDistance = current.dist + nodes[curLabel].neighbors[curNeighborLabel]
            if newDistance < curDistance:
                neighbor = predecessorTree.nodes[curNeighborLabel]
                if neighbor in pq:
                    pq.remove(neighbor)
                neighbor.dist = newDistance
                neighbor.depth = current.depth + 1
                neighbor.setParent(curLabel)
                pq.append(neighbor)
                heapify(pq)
        print("%d th loop" % (count))
        print(pq)
    
    # if curLabel == targetLabel:
    shortestPath = Path()
    currentN = targetLabel
    parentN = predecessorTree.getParentOf(currentN)
    while parentN is not None:
        e = Edge()
        e.fromNode = parentN
        e.toNode = currentN
        e.weight = nodes[parentN].neighbors[currentN]
        shortestPath.addFirst(e)
        currentN = parentN
        parentN = predecessorTree.getParentOf(currentN)

    return shortestPath    
                
    # return None
    
# graph = Graph()
# graph.getDataFromFile('tiny_graph_01.1.txt')

# sourceNode = "1"
# targetNode = "4"
    


# tree = shortestPathTree(graph, targetNode)

# for n in tree.nodes:
#      print(n)
#      print(tree.getParentOf(n))
#      print("------")

#tree = ShortestPathTree(root=targetNode)
    
# print(shortestPath(graph,sourceNode,targetNode))
