class Dijsktra:
    def shortestPathTree(graph, sourceLabel):
        nodes = graph.nodes
        if sourceLabel not in node.keys():
            raise Exception("Source is not in graph")

        predecessorTree = ShortestPathTree(sourcelabel)
        visited = {}
# this is priority queue - use heap function on python
        pq = []
        
        for nodelabel in nodes.keys():
            newNode = DijsktraNode(nodeLabel)
            newNode.dist = float("inf")
            newNode.depth = 99999
            predecessorTree.add(newNode)

        sourceNode = predecessorTree.nodes.get(predecessortree.getRoot())
        sourceNode.dist = 0
        sourceNode.depth = 0

        heappush(pq, sourceNode)

        count = 0

        while len(pq)>0:
            current = heappop(pq)
            currLabel = current.label


