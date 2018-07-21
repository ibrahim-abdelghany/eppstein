from eppstein import *
from dijsktra import *
from components import *
import timeit

def checkLoop(ksp):
    sb = []
    
    numEdges = len(ksp.edges)
    if numEdges > 0:
        for i in range(numEdges):
            if ksp.edges[i].fromNode in sb:
                return False
            else:
                sb.append(ksp.edges[i].fromNode)
    
        if ksp.edges[numEdges-1].toNode in sb:
            return False
    
    return True

def checkLineTransfer(graph,tree,kpath):
    extraDistForTransfer = 0
    lineDic = {}
    numTransfer = 0
    isAvailable = True
    
    lineDic[kpath.edges[0].fromNode] = graph.nodes[kpath.edges[0].fromNode].neighborsAvailableLines[kpath.edges[0].toNode]
    
    for i in range(len(kpath.edges)-1):
        cur = kpath.edges[i]
        nex = kpath.edges[i+1]
        
        availableLines = lineDic[cur.fromNode].intersection(graph.nodes[cur.toNode].neighborsAvailableLines[nex.toNode])
        
        if len(availableLines) > 0:
            lineDic[cur.toNode] = availableLines
        else:
            numTransfer += 1
            lineDic[cur.toNode] = graph.nodes[cur.toNode].neighborsAvailableLines[nex.toNode]
            if numTransfer > 4:
                isAvailable = False
                break
    
    return isAvailable, numTransfer, lineDic
        # if len(lineListAvailable) == 0:
        #     newDistance += 3
        #     lineListAvailable = lineListCurNeighbor
        #     if ksp.edges[i].fromNode in sb:
        #         return False
        #     else:
        #         sb.append(ksp.edges[i].fromNode)
    

def ksp(graph, sourceLabel, targetLabel, K):
    tree = shortestPathTree(graph.transpose(), targetLabel)
    sidetrackEdgeCostMap = computeSidetrackEdgeCosts(graph, tree)
    # Make indexes to give fast access to these heaps later */
    # Heap H_out(v) for every node v
    nodeHeaps = {}
    edgeHeaps = {}
    # Heap H_T(v) for every node v
    outrootHeaps = {}
    #dic of EppsteinArrayHeap
    arrayHeaps = {}

    hg = EppsteinHeap(sidetrack=Edge(fromNode=sourceLabel,toNode=sourceLabel,weight=0))
    
    #  Initialize the containers for the candidate k shortest paths and the actual found k shortest paths
    ksp = []
    result = []
    #Heap
    pathPQ = []
    heappush(pathPQ, EppsteinPath(heap=hg, prefPath=-1, cost=tree.nodes[sourceLabel].dist))
    
    i = 0
    # for i in range(0,K):
    while True:
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
        kpath = kpathImplicit.explicitPath(graph, ksp, tree);

        #  Add explicit path to the list of K shortest paths
        
        ksp.append(kpath)
        
        #  Push the (up to 3) children of this path within the Eppstein heap onto the priority queue
        addExplicitChildrenToQueue(kpathImplicit, ksp, pathPQ)
        

        # Check for the existence of a potential fourth child, known as a "cross edge", to push onto the queue.
        # This heap edge/child does not need to be explicitly represented in the Eppstein heap because it is easy
        #         to check for its existence. 
        # If the cross-edge child does not exist, try building its part of the heap, then check again.
        # Note: this is where all of the heap building happens in the lazy version of Eppstein's algorithm.
        if kpathImplicit.heap.sidetrack.toNode not in outrootHeaps.keys():
            buildHeap(kpathImplicit.heap.sidetrack.toNode, graph, sidetrackEdgeCostMap, nodeHeaps, edgeHeaps, outrootHeaps, arrayHeaps, tree)
        addCrossEdgeChildToQueue(outrootHeaps, kpathImplicit, i, ksp, pathPQ);
        
        if checkLoop(kpath):
            isAvailable, numTransfer, lineDic = checkLineTransfer(graph,tree,kpath)
            if isAvailable:
                i+=1
                kpath.totalCost += 3*numTransfer
                kpath.numTransfer = numTransfer
                kpath.lineDic = lineDic
                heappush(result,kpath)
        
        if i == K: 
            break

        # // Return the set of k shortest paths
    return result;

def buildHeap(nodeLabel, graph, sidetrackEdgeCostMap, nodeHeaps, edgeHeaps, outrootHeaps, arrayHeaps, tree):
    computeOutHeap(nodeLabel, graph, sidetrackEdgeCostMap, nodeHeaps, edgeHeaps)
    
    if nodeLabel == tree.root:
        currentArrayHeap = EppsteinArrayHeap()
    else:
        if tree.getParentOf(nodeLabel) not in outrootHeaps.keys(): 
            # Case 1: H_T(nextT(v)) has not been built, so build it
            buildHeap(tree.getParentOf(nodeLabel), graph, sidetrackEdgeCostMap,  nodeHeaps, edgeHeaps, outrootHeaps, arrayHeaps, tree)
        
        # Case 2: H_T(nextT(v)) has been built
        currentArrayHeap = arrayHeaps.get(tree.getParentOf(nodeLabel));    
    try:
        sidetrackHeap = nodeHeaps[nodeLabel]
    except KeyError as ke:
        sidetrackHeap = None
    if not(sidetrackHeap is None):
        currentArrayHeap = currentArrayHeap.clone()
        currentArrayHeap.addOutRoot(sidetrackHeap)
    
    arrayHeaps[nodeLabel] = currentArrayHeap
    
    currentHeap = currentArrayHeap.toEppsteinHeap2()
    if not(currentHeap is None):
        outrootHeaps[nodeLabel] = currentHeap
        
def testSimpleData():
    graph = Graph()
    
    K = 10
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
    targetNode = "106410";
    K = 20;
    
    graph.getDataFromBusFile('bus.csv')
    
    print("Reading data from file is complete")
    print("Computing %d shortest paths from data" %(K))
    
    start = timeit.default_timer()
    
    kspList = ksp(graph, sourceNode, targetNode, K)
    
    stop = timeit.default_timer()
    
    n = 1
    while len(kspList)>0:
        p = heappop(kspList)
        print("%d) %s" % (n,p))
        n+=1
        
    print("Elapsed time : %f" %(stop - start))

if __name__ == "__main__":
    testBusData()