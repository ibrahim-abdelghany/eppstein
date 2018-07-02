from heapq import heappush heappop

def shortestPathTree(graph, sourceLabel):
    nodes = graph.nodes
    if sourceLabel not in nodes.keys():
        raise Exception("source not in nodes")

    visited = set([])
    predecessorTree = ShortestPathTree(sourceLabel)
    pq = []

    for nodeLabel in nodes.keys():
        node = dijsktraNode(nodeLabel)
        node.dist = flat("inf")
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
        visited.append(current)
        count += 1

        neighbors = nodes.get(curLabel).neighbors
        for curNeighborLabel in neighbors.keys():
            neighborNode = predecessorTree.nodes[curNeighborLabel]

