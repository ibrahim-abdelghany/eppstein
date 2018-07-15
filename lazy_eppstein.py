from eppstein import *
from dijsktra import *
from components import *

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
    #Heap
    pathPQ = []
    heappush(pathPQ, EppsteinPath(heap=hg, prefPath=-1, cost=tree.nodes[sourceLabel].dist))
    
    for i in range(0,K):
        if len(pathPQ) < 1:
            break
            #  Get the next shortest path, which is implicitly represented as:
            #     1) Some shorter path, p, from s (source) to t (target)
            #     2) A sidetrack edge which branches off of path p at node u, and points to node v
            #     3) The shortest path in the shortest path tree from node v to t 
        print("PathPQ")
        print(pathPQ)
        kpathImplicit = heappop(pathPQ)
       
        #  Convert from the implicit path representation to the explicit path representation
        kpath = kpathImplicit.explicitPath(ksp, tree);

        #  Add explicit path to the list of K shortest paths
        ksp.append(kpath);

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

        # // Return the set of k shortest paths
    return ksp;

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