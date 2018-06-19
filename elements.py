from heapq import heappush, heappop

class Vertex:
    def __init__(self):
        self.edge2path = None
        self.distance = -1
        self.relatedEdges = []
        self.label = ''
        self.sortType = 0

    def nextVertex(self):
        if self.edge2path is None:
            return None
        else:
            return edge2path.head

    def __str__(self):
        return self.label

    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        if other is None:
            return False
        return self.label == other.label

    def __hash__(self):
        return hash(str(self))

    def getWeight(self):
        if self.sortType == 0:
            return self.time
        elif self.sortType == 1:
            return self.price

class Edge:
    def __init__(self):
        self.time = -1
        self.distance = -1
        self.price = 0
        self.tp_type = 'None'
        self.lineNum = -1
        self.sortType = 0

        self.head = None
        self.tail = None
        self.weight = 0
    
    def __lt__(self, other):
        if self.getWeight() == other.getWeight():
            return self.getSubWeight() < other.getSubWeight()
        return self.getWeight() < other.getWeight()

    def __gt__(self,other):
        return other.__lt__(self)

    def __eq__(self, other):
        if other is None:
            return False
        return self.getWeight() == other.getWeight() and self.getSubWeight() == other.getSubWeight()

    def __ne__(self,other):
        return not self.__eq__(other)

    def __str__(self):
        return "%s -- %d --> %s" % (self.tail,self.weight,self.head)

    def __repr__(self):
        return self.__str__()

    def delta(self):
        return self.getWeight() + self.head.getWeight() - self.tail.getWeight()

    def isSidetrackOf(self, v):
        return self.tail == v and self != v.edge2path and self.weight >= 0

    def getWeight(self):
        if self.sortType == 0:
            return self.time
        elif self.sortType == 1:
            return self.price

    def getSubWeight(self, sortType):
        if sortType == 0:
            return self.price
        elif sortType == 1:
            return self.time

class Path(list):
    def __init__(self):
        super().__init__()

    def isValid(self):
        return len(self) > 0

    def getVertexNames(self):
        if not isValid():
            return "(Empty)"

        strList = []
        strList.append(self[0].tail)

        for e in self:
            strList.append(",")
            strList.append(e.head)

        return ''.join(strList)

    def getWeight(self):
        total = 0
        for e in self:
            total += e.getWeight()

        return total

    def getDeltaWeight(self):
        total = 0
        for e in self:
            total += e.delta()

        return total

    def __str__(self):
        if not isValid():
            return "(Empty)"

        strList = []
        for e in self:
            strList.append(e.delta())
            strList.append(',')

        if len(strList) > 0:
            del strList[len(strList)-1]

        return ''.join(strList)

class SPNode:
    def __init__(self):
        self.edge = Edge()
        self.weight = 0

    def __str__(self):
        return "["+self.weight+"]"

    def __repr__(self):
        return self.edge.__str__()

    def __lt__(self, other):
        if self.getWeight() == other.getWeight():
            return self.getSubWeight() < other.getSubWeight()
        return self.getWeight() < other.getWeight()

    def __gt__(self,other):
        return other.__lt__(self)

    def __eq__(self, other):
        return self.getWeight() == other.getWeight() and self.getSubWeight() == other.getSubWeight()
    def __ne__(self,other):
        return not self.__eq__(other)

    def getWeight(self):
        return self.weight

    def getSubWeight(self):
        return 0

class STNode:
    def __init__(self):
        self.sidetracks = Path()
        self.weight = 0

    def __str__(self):
        return "["+str(self.weight)+"]"

    def __lt__(self, other):
        if self.getWeight() == other.getWeight():
            return self.getSubWeight() < other.getSubWeight()
        return self.getWeight() < other.getWeight()

    def __gt__(self,other):
        return other.__lt__(self)

    def __eq__(self, other):
        return self.getWeight() == other.getWeight() and self.getSubWeight() == other.getSubWeight()

    def __ne__(self,other):
        return not self.__eq__(other)

class Graph:
    def __init__(self):
        self.vertexList = []
        self.pathHeap = []
        self.ready = False
        self.origin = None
        self.dest = None

    def createVertices(self,_vertices):
        self.ready = False
        labelList = _vertices.split(",")

        for l in labelList:
            if len(l) == 0:
                continue
            v = self.getVertex(self.vertexList,l)
            
            if v is None:
                v = Vertex()
                v.label = l
                self.vertexList.append(v)

        return True

    def createEdges(self,_tails, _heads, _weight):
        tails = self.getVertices(_tails)
        heads = self.getVertices(_heads)

        if tails is None or heads is None:
            return False
        if len(tails) != len(heads):
            return False

        for i in range(len(tails)):
            e = Edge()
            e.tail = tails[i]
            e.head = heads[i]
            e.weight = _weight
            tails[i].relatedEdges.append(e)
            if tails[i] != heads[i]:
                heads[i].relatedEdges.append(e)

        self.ready = False

        return True
    
    def getVertex(self, vertextList, name):
        for v in vertextList:
            if v.label == str(name):
                return v
        else:
            return None

    def getVertices(self,_verticeLabels):
        labels = _verticeLabels.split(',')
        result = []
        if len(labels) == 0:
            return None

        for label in labels:
            v = self.getVertex(self.vertexList,label)
            if v is None:
                return None
            result.append(v)

        return result

    def findShortestPath(self,s,t):
        self.ready = False
        
        self.origin = self.getVertex(self.vertexList,s)
        self.dest = self.getVertex(self.vertexList,t)

        self.buildShortestPathTree()

        self.buildSidetracksHeap()

        self.ready = True

        return self.findNextShortestPath()

    def findNextShortestPath(self):
        if not self.ready:
            return Path()

        node = heappop(self.pathHeap)

        if node is None:
            return Path()

        print(node)

        return self.rebuildPath(node.sidetracks)

    def resetGraphState(self):
        for v in self.vertexList:
            v.edge2path = None
            v.distance = -1

    def buildShortestPathTree(self):
        self.resetGraphState()

        v = self.dest
        v.distance = 0

        fringe = []

        while True:
            if v is not None:
                for e in v.relatedEdges:
                    if e.head ==v and e.weight >= 0:
                        _n = SPNode()
                        _n.edge = e
                        _n.weight = e.weight + e.head.distance
                        heappush(fringe,_n)
           
            print(len(fringe))
            try:
                node = heappop(fringe)
                print(node)
            except IndexError as ie:
                print(ie)
            finally:
                break

            e = node.edge;
            v = e.tail
            print(v)
            if v.distance == -1:
                v.distance = e.weight + e.head.distance
                v.edge2path = e
                print(v)
                print(v.edge2path)
            else:
                v = None
    
    def buildSidetracksHeap(self):
        self.pathHeap = []
        empty = Path()
        stNode = STNode()
        stNode.pathHeap = empty
        heappush(self.pathHeap,stNode)

        self.addSidetracks(empty, self.origin)

    def addSidetracks(self, _p, _v):
        for e in _v.relatedEdges:
            if e.isSidetrackOf(_v) and (e.head.edge2path is not None or e.head == self.dest):
                p = Path()
                p.addRange(_p)
                heappush(p,e)
                heappush(self.pathHeapST_Node(p))

                if e.head != _v:
                    addSidetracks(p, e.head)
        
        if _v.nextVertex() != None:
            addSidetracks(_p, _v.nextVertex())

    def rebuildPath(self, _sidetracks):
        path = Path()
        v = self.origin
        i = 0

        while v != None:
            if i<len(_sidetracks) and _sidetracks[i].tail == v:
                heappush(path,_sidetracks[i])
                v = _sidetracks[i].head
                i+=1
            else:
                if v.edge2path is None:
                    break
                heappush(path,v.edge2path)
                v = v.nextVertex()

        return path


