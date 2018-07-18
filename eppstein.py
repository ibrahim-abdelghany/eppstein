from components import *
from dijsktra import *

'''
     * Computes the K shortest paths (allowing cycles) in a graph from node s to node t in graph G using Eppstein's
     * algorithm. ("Finding the k Shortest Paths", Eppstein)
     *
     * Some explanatory notes about how Eppstein's algorithm works:
     * - Start with the shortest path in the graph from s to t, which can be calculated using Dijkstra's algorithm.
     * - The second shortest path must diverge away from this shortest path somewhere along the path at node u, with
     * edge (u,v) known as a "sidetrack" edge. It then follows the shortest path from v to t.
     * - In general, let T represent the shortest path tree rooted at target node t, containing for each node v, an edge
     * (v,w) to the parent of node v (w) along its shortest path to node t.
     * - All other edges (u,v) in the graph are "sidetrack" edges. In other words, this includes any edge (u,v) for
     * which node v is not on the shortest path from u to t.
     * - All paths from s to t can be uniquely represented by the sequence of sidetrack edges that appear in the path.
     * - All non-sidetrack edges in the path are represented implicitly in the shortest path tree, T.
     * - All paths from s to t can be represented in a tree, H, where the root node has no sidetrack edges, its children
     * are all of the possible one sidetrack paths, the children of each of these sidetracks are the paths with a
     * second sidetrack edge, and so on.
     * - Each path from s to t corresponds to a path from the root of H to some descendant node in H.
     * - The tree is a heap, as each additional sidetrack creates a new path whose cost is either the same as or greater
     * than the cost of its parent path.
     * - One possible way to find the k shortest paths, then, is to maintain a priority queue of candidate paths where
     * once a path is pulled from the priority queue, its children in the heap are added to the priority queue.
     * - Two observations about this heap:
     *   - Each node in H has at most O(E) children, where E is the number of edges in the graph.
     *   - If G has looped paths, then this heap is infinite.
     * - Eppstein's algorithm is basically a scheme for re-organizing this heap so that each heap node has at most 4
     * children, which is O(1) instead of O(E) - performing potentially much less work for each path found.
     * - This re-organized heap R has the following form, in terms of the original heap H:
     *   - For each heap node in H, select its best child, B.
     *   - This child is known as a "cross edge" child.
     *   - Child B has N siblings in H.
     *   - These N siblings are removed from its parent in heap H and are made instead to be descendants of B in heap R.
     *   - The best child, B, has up to three of its siblings placed as direct children (along with its own best "cross
     *   edge" child in the original heap H, this yields a total of four possible children in R).
     *   - Among its sibling-children, one of its children in R is the root of a new binary heap containing all of the
     *   siblings of B in H that are sidetrack edges directed away from the same node in G.
     *   - All of the remaining siblings of B in H are placed in a binary heap with B as the root in R.
     *   - Because this is a binary heap, B has up to two children of this type.
     *   - Thus, B has at most four children in R. Any node in R that is a "cross-edge" child has up to four children,
     *   whereas other nodes fall inside binary heaps and are limited to having at most two children.
     *
     * @param graph         the graph on which to compute the K shortest paths from s to t
     * @param sourceLabel   the starting node for all of the paths
     * @param targetLabel   the ending node for all of the paths
     * @param K             the number of shortest paths to compute
     * @return              a list of the K shortest paths from s to t, ordered from shortest to longest
     */
'''
def ksp(graph, sourceLabel, targetLabel, K):
    tree = shortestPathTree(graph.transpose(), targetLabel)
    
    sideTrackEdgeCostMap = computeSidetrackEdgeCosts(graph, tree)
    
    
    # Make indexes to give fast access to these heaps later */
    # Heap H_out(v) for every node v
    nodeHeaps = {}
    edgeHeaps = {}
    # Heap H_T(v) for every node v
    outrootHeaps = {}
    
    #   COMPUTE EPPSTEIN HEAP, Part 1: Compute sub-heap H_out(v) for each node v.
    #         -- H_out(v) is a heap of all of the outgoing sidetrack edges of v. 
    for nodeLabel in graph.nodes.keys():
        computeOutHeap(nodeLabel, graph, sideTrackEdgeCostMap, nodeHeaps, edgeHeaps)
        # /* COMPUTE EPPSTEIN HEAP, Part 2: Compute sub-heap H_T(v) for each node v.
        #     -- H_T(v) is a heap of all of the "best" sidetrack edges for each node on the shortest path from v to T.
        #     -- H_T(v) is computed by adding the lowest cost sidetrack edge of v to heap H_T(nextT(v)),
        #     where nextT(v) is the parent node of node v in the shortest path tree rooted at the target node T.
        #     -- Therefore, can compute H_T(v) recursively. But instead of a top-down recursion, we will compute each
        #     H_T(v) bottom-up, starting with the root node of the tree, T.
        #     -- To facilitate bottom-up computation, reverse the edges of the shortest path tree so each node points to
        #     its children instead of its parent. */
    reversedSPT = Graph();
    for label, node in tree.nodes.items():
        try:
            reversedSPT.addEdge(fromNode=node.getParent(),toNode=node.label,weight=graph.getNode(node.label).neighbors[node.getParent()]);
        except KeyError as ke:
            pass
    
        
    # /* Use a depth-first search from node T to perform the bottom-up computation, computing each H_T(v) given
    #     H_T(nextT(v)). */
    #  Create the initial (empty) heap for the root node T to build from.
    rootArrayHeap = EppsteinArrayHeap();
    #  Perform the DFS (recursively initiating additional depth-first searches for each child)
    recursiveOutrootHeaps(targetLabel, rootArrayHeap, nodeHeaps, outrootHeaps, reversedSPT);

    #  Create a virtual/dummy heap that is the root of the overall Eppstein heap. It represents the best path from
    #  the source node to the target node, which does not involve any sidetrack edges.
    hg = EppsteinHeap(sidetrack=Edge(fromNode=sourceLabel,toNode=sourceLabel,weight=0))
    
    #  Initialize the containers for the candidate k shortest paths and the actual found k shortest paths
    ksp = []
    #Heap
    pathPQ = []
        
    #  Place root heap in priority queue
    heappush(pathPQ, EppsteinPath(heap=hg, prefPath=-1, cost=tree.nodes[sourceLabel].dist))
       
        #  Pop k times from the priority queue to determine the k shortest paths 
    for i in range(0,K):
        if len(pathPQ) < 1:
            break
            #  Get the next shortest path, which is implicitly represented as:
            #     1) Some shorter path, p, from s (source) to t (target)
            #     2) A sidetrack edge which branches off of path p at node u, and points to node v
            #     3) The shortest path in the shortest path tree from node v to t 
        # print("PathPQ")
        # print(pathPQ)
        kpathImplicit = heappop(pathPQ)
       
        #  Convert from the implicit path representation to the explicit path representation
        kpath = kpathImplicit.explicitPath(ksp, tree);

        #  Add explicit path to the list of K shortest paths
        ksp.append(kpath);

        #  Push the (up to 3) children of this path within the Eppstein heap onto the priority queue
        addExplicitChildrenToQueue(kpathImplicit, ksp, pathPQ)
        

        #  Check for the existence of a potential fourth child, known as a "cross edge", to push onto the queue.
        #     This heap edge/child does not need to be explicitly represented in the Eppstein heap because it is easy
        #     to check for its existence. 
        addCrossEdgeChildToQueue(outrootHeaps, kpathImplicit, i, ksp, pathPQ);

        # // Return the set of k shortest paths
    return ksp;
'''
     * Compute the set of sidetrack edge costs.
     *
     * Each sidetrack edge (u,v) is an edge in graph G that does not appear in the shortest path tree, T.
     * For every sidetrack edge (u,v), compute S(u,v) = w(u,v) + d(v) - d(u), where w(u,v) is the cost of edge (u,v);
     * and d(v) is the cost of the shortest path from node v to the target.
     *
     * @param graph     the graph on which to compute the K shortest paths from s to t
     * @param tree      the shortest path tree, T, rooted at the target node, t
     * @return
'''
def computeSidetrackEdgeCosts(graph, tree):
    sideTrackEdgeCostMap = {}
    edgeList = graph.getEdgeList()
    
    for e in edgeList:
        # Check to see if the target node is reachable from the outgoing vertex of the current edge,
        # and check to see if the current edge is a sidetrack edge. If so, calculate its sidetrack cost.
        tp = tree.getParentOf(e.fromNode)
        if tp == None or not tp == e.toNode:
            sidetrackEdgeCost = e.weight + tree.nodes[e.toNode].dist - tree.nodes[e.fromNode].dist
            if len(tree.nodes[e.fromNode].lines & e.availableLines) < 1:
                sidetrackEdgeCost += 3
            hashStr = e.fromNode+","+e.toNode
            sideTrackEdgeCostMap[hashStr] = sidetrackEdgeCost 
    
    return sideTrackEdgeCostMap
    
# /**
#      * Compute sub-heap H_out(v) for node v.
#      *
#      * @param nodeLabel             node v
#      * @param graph                 the graph, G, on which to compute the K shortest paths from s to t
#      * @param sidetrackEdgeCostMap  the cost of each sidetrack edge in G
#      * @param nodeHeaps             an index/hash table of heap H_out(v), for each node v in the graph
#      * @param edgeHeaps             an index/hash table of heaps H_out(v), but indexed by sidetrack edge
#      */
def computeOutHeap(nodeLabel, graph, sidetrackEdgeCostMap, nodeHeaps, edgeHeaps):
    node = graph.getNode(nodeLabel)
    # // This list holds the 2nd through last sidetrack edges, ordered by sidetrack cost
    sidetrackEdges = []
    bestSidetrack = None
    minSidetrackCost = float("inf");
    # // Iterate over the outgoing edges of v
    for neighbor in node.getAdjacencyList():
        edgeLabel = nodeLabel+","+neighbor
        # // Check to see if the current edge is a sidetrack edge
        if edgeLabel in sidetrackEdgeCostMap.keys():
            sidetrackEdgeCost = sidetrackEdgeCostMap[edgeLabel]
            # // Check to see if the current sidetrack edge has the lowest cost discovered so far for node v
            if sidetrackEdgeCost < minSidetrackCost:
                # // If there was a previously-known best sidetrack edge, add it to the list of non-best
                # // sidetrack edges
                if bestSidetrack is not None:
                    sidetrackEdges.append(bestSidetrack)
                
                # // Set the new best (lowest cost) sidetrack edge to be the current one
                bestSidetrack = Edge(fromNode=nodeLabel, toNode=neighbor, weight=node.neighbors[neighbor])
                minSidetrackCost = sidetrackEdgeCost
            # // If current sidetrack edge is not the one with the lowest cost, add it to the list of non-best
            # // sidetrack edges
            else:
                sidetrackEdges.append(Edge(fromNode=nodeLabel, toNode=neighbor, weight=node.neighbors[neighbor]))
    
    # // If v was found to have at least one outgoing sidetrack edge...
    if bestSidetrack is not None:
        # // ...make a heap of the outgoing sidetrack edges of v, with the lowest-cost sidetrack edge put as the
        # // root
        bestSidetrackHeap = EppsteinHeap(sidetrack=bestSidetrack,sidetrackCost=sidetrackEdgeCostMap[bestSidetrack.fromNode+","+bestSidetrack.toNode])

        # // Make another heap (a binary heap) out of the rest of the sidetrack edges of v
        arrayHeap = EppsteinArrayHeap()
        if len(sidetrackEdges) > 0:
            bestSidetrackHeap.numOtherSidetrack = bestSidetrackHeap.numOtherSidetrack+1
            for edge in sidetrackEdges:
                sidetrackHeap = EppsteinHeap(sidetrack=edge, sidetrackCost=sidetrackEdgeCostMap[edge.fromNode+","+edge.toNode])
                edgeHeaps[edge.fromNode+","+edge.toNode] = sidetrackHeap
                arrayHeap.add(sidetrackHeap)

            # // Add the binary heap of 2nd-through-last lowest cost sidetrack edges as a child (the only child)
            # // of the lowest-cost sidetrack edge, forming the overall heap H_out(v)
            bestSidetrackHeap.addChild(arrayHeap.toEppsteinHeap())

        # // Index H_out(v) by node v, for easy access later
        nodeHeaps[nodeLabel] =  bestSidetrackHeap
        # // Index H_out(v) by its lowest cost sidetrack edge, for easy access later
        edgeHeaps[bestSidetrack.fromNode+","+bestSidetrack.toNode] = bestSidetrackHeap
    

    # /**
    #  * Push the (up to 3) children (within the Eppstein heap) of the given (kth) path, onto the priority queue.
    #  *
    #  * @param kpathImplicit     implicit representation of the (kth) path
    #  * @param ksp               list of shortest paths found so far
    #  * @param pathPQ            priority queue of candidate paths
    #  */
def addExplicitChildrenToQueue(kpathImplicit, ksp, pathPQ):
    kpathCost = kpathImplicit.cost;
    # print("[addExplicitChildren method]")
    # print("kpathImplicit.heap.children")
    # print(kpathImplicit.heap.children)
    for childHeap in kpathImplicit.heap.children:
        # // Get the index of the previous shorter path off of which this candidate sidetracks/branches
        prefPath = kpathImplicit.prefPath

        # // Calculate the path cost of the new child/candidate
        candidateCost = ksp[prefPath].totalCost + childHeap.sidetrackCost

        # // Add the child/candidate to the priority queue
        candidate = EppsteinPath(heap=childHeap, prefPath=prefPath, cost=candidateCost)
        # print("Candidate")
        # print(candidate)
        heappush(pathPQ,candidate)
        
    # /**
    #  *
    #  * @param outrootHeaps      an index of heaps H_T(v) for each node v
    #  * @param kpathImplicit     implicit representation of the (kth) path
    #  * @param prefPath          the index k of the path off which this cross-edge child sidetracks
    #  * @param ksp               list of shortest paths found so far
    #  * @param pathPQ            priority queue of candidate paths
    #  */
def addCrossEdgeChildToQueue(outrootHeaps, kpathImplicit, prefPath, ksp, pathPQ):
    # print("addCrossEdgeChildToQueue")
    # print("kpathImplicit.heap.sidetrack.toNode")
    # print(kpathImplicit.heap.sidetrack.toNode)
    # print(outrootHeaps)
    if kpathImplicit.heap.sidetrack.toNode in outrootHeaps.keys():
        childHeap = outrootHeaps[kpathImplicit.heap.sidetrack.toNode]

        # // Calculate the path cost of the new child/candidate
        candidateCost = ksp[prefPath].totalCost + childHeap.sidetrackCost
        
        # print("ChildHeap")
        # print(childHeap)

        # // Add the child/candidate to the priority queue
        candidate = EppsteinPath(heap=childHeap, prefPath=prefPath, cost=candidateCost);
        # print("Candidate")
        # print(candidate)
        heappush(pathPQ,candidate)
        
    # /**
    #  * Generate sub-heap H_T(v) for node v and its children in the shortest path tree T, using a recursive depth-first
    #  * search over the transpose graph, T', of the shortest path tree, T.
    #  *
    #  * The transpose graph is necessary because tree T is represented with pointers to parents instead of with pointers
    #  * to children.
    #  *
    #  * @param nodeLabel         node v
    #  * @param currentArrayHeap  the heap of v's parent in the shortest path tree; H_T(nextT(v))
    #  * @param nodeHeaps         an index/hash table of heap H_out(v), for each node v in the graph
    #  * @param outrootHeaps      an index/hash table of heap H_T(v), for each node v in the graph
    #  * @param reversedSPT       the transpose graph, T', of the shortest path tree, T
    #  */
def recursiveOutrootHeaps(nodeLabel, currentArrayHeap, nodeHeaps, outrootHeaps, reversedSPT):
        # // Get H_out(v)
    try:
        sidetrackHeap = nodeHeaps[nodeLabel]
    except KeyError as ke:
        print(ke)
        sidetrackHeap = None

    # // Check to see if node v (nodeLabel) has a sidetrack edge
    if sidetrackHeap is not None:
        # // If so, need to add its best sidetrack edge to H_T(nextT(v))
        # // H_T(nextT(v)) is in variable currentArrayHeap, which was passed onto v when this function was called by
        # // nextT(v)

        # // The goal of Eppstein's algorithm is to re-use heap structures where possible
        # // When adding the best sidetrack edge of v to heap H_T(nextT(v)) to form H_T(v), this means that when
        # // the best sidetrack of v is added to H_T(nextT(v)), we need to make a new copy of all of the heap nodes
        # // on the path from the position where the sidetrack is added in the heap, to the root of the heap. All
        # // other heap nodes have the same structure in H_T(v) as in H_T(nextT(v)) and can simply be pointed to.

        # // Give H_T(v) a new set of pointers to the sub-heap nodes in H_T(nextT(v))
        currentArrayHeap = currentArrayHeap.clone()

        # // Add the best sidetrack edge of v to heap H_T(v) (place the sidetrack edge in the next unoccupied space in
        # // the heap and bubble it up the tree to its rightful position) and make all new copies of the sub-heap
        # // nodes that fall on the path where the new edge is added in the heap, to the root of the heap
        
        currentArrayHeap.addOutRoot(sidetrackHeap)
        # print("currentArrayHeap.arrayHeap");
        # print(currentArrayHeap.arrayHeap)
    

    # // Convert from an array representation of the heap (EppsteinArrayHeap), which is convenient for accessing and
    # // manipulating a binary heap, to a pointer representation (EppsteinHeap), which is consistent with the overall
    # // heap, which is not strictly a binary heap since some nodes can have up to 4 children.
    currentHeap = currentArrayHeap.toEppsteinHeap2()
    # print("CurrentHeap")
    # print(currentHeap)

    # // If v has any children (so H_T(v) exists), index heap H_T(v) in a list of heaps for fast access later
    if currentHeap is not None:
        outrootHeaps[nodeLabel] = currentHeap
    
    # print(reversedSPT.getNode(nodeLabel).neighbors.keys())

    # // Continue the depth-first search (recursively initiating additional depth-first searches for each child)
    for neighbor in reversedSPT.getNode(nodeLabel).neighbors.keys():
        recursiveOutrootHeaps(neighbor, currentArrayHeap, nodeHeaps, outrootHeaps, reversedSPT);
        
    
class EppsteinHeap:
    def __init__(self, **kargs):
        self.children = []
        self.sidetrackCost = 0.0
        self.numOtherSidetrack = 0
        self.sidetrack = None
        for key, value in kargs.items():
            if key == 'sidetrack':
                self.sidetrack = value
            elif key == 'sidetrackCost':
                self.sidetrackCost = value
            elif key == 'children':
                self.children = value
            elif key == 'numOtherSidetrack':
                self.numOtherSidetrack = value
    
    def __lt__(self,other):
        if self is None:
            print("self is None")
        elif other is None:
            print("other is None")
        return self.sidetrackCost < other.sidetrackCost
    
    def __gt__(self,other):
        return other.__lt__(self)
    
    def addChild(self, child):
        self.children.append(child)
    
    def clone(self):
        newChildren = []
        # is that needed to make new child(EppsteinHeap) for this method(clone)?
        for child in self.children:
            newChildren.append(child)
        
        eh = EppsteinHeap(sidetrack=self.sidetrack, sidetrackCost=self.sidetrackCost, children=newChildren, numOtherSidetrack=self.numOtherSidetrack)
        return eh
    def __repr__(self):
        return self.sidetrack.__str__()
    
class EppsteinArrayHeap:
    def __init__(self):
        self.arrayHeap = []
    
    def getParentIndex(self, i):
        return int((i-1)/2)
        
    def add(self, eppsteinHeap):
        heappush(self.arrayHeap, eppsteinHeap)
        
    def addOutRoot(self, heap):
        current = len(self.arrayHeap)
        
        while current > 0:
            parent = self.getParentIndex(current)
            newHeap = self.arrayHeap[parent].clone()
            self.arrayHeap.insert(parent,newHeap)
            current = parent
        
        heappush(self.arrayHeap, heap)
        # print("arrayHeap")
        # print("------")
        # for h in self.arrayHeap:
        #     print(h)
        # print("------")
    
    def toEppsteinHeap(self):
        heapSize = len(self.arrayHeap)
        if heapSize == 0:
            return None
            
        eh = self.arrayHeap[0]
        
        for i in range(heapSize):
            h = self.arrayHeap[i]
            self.arrayHeap[self.getParentIndex(i)].addChild(h)

        return eh
        
    #  Convert from an array representation of a binary heap to a pointer representation of a binary heap, which can fit
    #  consistently within an overall non-binary heap.
    def toEppsteinHeap2(self):
        arrayHeap = self.arrayHeap
        current = len(arrayHeap)-1
        
        if current == -1:
            return None
            
        while current >= 0:
            childHeap = arrayHeap[current]
            while len(childHeap.children) > childHeap.numOtherSidetrack:
                childHeap.children.pop(len(childHeap.children)-1)
            
            child1 = current * 2 + 1
            child2 = current * 2 + 2

            if child1 < len(arrayHeap):
                arrayHeap[current].addChild(arrayHeap[child1])
            
            if child2 < len(arrayHeap): 
                arrayHeap[current].addChild(arrayHeap[child2])
            
            if current > 0:
                current = self.getParentIndex(current)
            
            else:
                current = -1;

        return arrayHeap[0]
    
    def clone(self):
        clonedArrayHeap = EppsteinArrayHeap()
        
        for heap in self.arrayHeap:
            clonedArrayHeap.add(heap)
        
        return clonedArrayHeap


'''
 * Data structure for representing a source-target path implicitly inside the priority queue of candidate k shortest
 * paths during the execution of Eppstein's algorithm.
'''
class EppsteinPath:
    '''
    EppsteinHeap heap; // pointer to the heap node and last sidetrack edge in this candidate path
    int prefPath; // index of the shorter path that this path sidetracks from
    Double cost; // the total cost of the path
    '''
    def __init__(self, **kargs):
        self.heap=None
        self.prefPath = -1
        self.cost = 0
        for key,value in kargs.items():
            if key == "heap":
                self.heap = value
            elif key == "prefPath":
                self.prefPath = value
            elif key == "cost":
                self.cost = value
    '''
    // Convert from the implicit representation of the path to an explicit listing of all of the edges in the path
    // There are potentially three pieces to the path:
    // 1) the path from node s (source) to node u in the parent path
    // 2) the sidetrack edge (u,v)
    // 3) the shortest path (in the shortest path tree) from node v to node t (target)
    '''
    def explicitPath(self, graph, ksp, tree):
        explicitPath = Path()
        heap = self.heap
        
        # if path is not the shortest path in the graph
        if self.prefPath >= 0:
            # // Get the explicit representation of the shorter parent path that this path sidetracks from
            explicitPrefPath = ksp[self.prefPath]
            
            print(explicitPrefPath)
            
            # // 1a) Identify the s-u portion of the path
            # // Identify and add the segment of the parent path up until the point where the current path sidetracks off
            # // of it.
            # // In other words, if (u,v) is the sidetrack edge of the current path off of the parent path, look for the
            # // last instance of node u in the parent path.
            edges = explicitPrefPath.edges
            lastEdgeNum = -1
            heapSidetrack = self.heap.sidetrack
            i = len(edges)-1
            while i >= 0:
                currentEdge = edges[i]
                if currentEdge.toNode == heapSidetrack.fromNode:
                    lastEdgeNum = i
                    break
                i-=1
            
            # // 1b) Add the s-u portion of the path
            # // Copy the explicit parent path up to the identified point where the current/child path sidetracks
            explicitPath = Path()
            for i in range(0,lastEdgeNum+1):
                explicitPath.add(edges[i])
            
            # // 2) Add the (u,v) portion of the path
            # // Add the last sidetrack edge to the explicit path representation
            explicitPath.add(heap.sidetrack)
        # // 3) Add the v-t portion of the path
        # // Add the shortest path from v (either the source node, or the incoming node of the sidetrack edge associated
        # // with the current path) to the explicit path representation
        current = heap.sidetrack.toNode
        
        while not current == tree.root:
            nextN = tree.getParentOf(current)
            edgeWeight = tree.nodes[current].dist - tree.nodes[nextN].dist
            explicitPath.add(Edge(fromNode=current,toNode=nextN,weight=edgeWeight))
            current = nextN
        
        return explicitPath
        
    def __lt__(self,other):
        return self.cost < other.cost
    
    def __gt__(self,other):
        return other.__lt__(self)
    
    def __eq__(self,other):
        return self.cost == other.cost
    
    def __ne__(self,other):
        return not self.__eq__(other)
        
    def __repr__(self):
        return 'heap : ' + str(self.heap) + ", prefPath : " + str(self.prefPath) +'cost : ' + str(self.cost)

def testSimpleData():
    graph = Graph()
    
    K = 2
    graph.getDataFromFile('tiny_graph_01.1.txt')
    print("Reading data from file is complete")
    print("Computing %d shortest paths from data")
    
    sourceNode = "0"
    targetNode = "3"
    
    kspList = ksp(graph, sourceNode, targetNode, K)
    
    n = 1
    for p in kspList:
        print("%d) %s" % (n,p))
        n+=1
        
def testBusData():
    graph = Graph()
    
    sourceNode = "105013";
    targetNode = "109157";
    K = 20;
    
    graph.getDataFromBusFile('bus.csv')
    print("Reading data from file is complete")
    print("Computing %d shortest paths from data" %(K))
    
    kspList = ksp(graph, sourceNode, targetNode, K)
    
    n = 1
    for p in kspList:
        print("%d) %s" % (n,p))
        n+=1
if __name__ == "__main__":
    testBusData()