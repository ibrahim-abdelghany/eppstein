from elements import *

g = Graph()

#make simple
#g.createVertices("S,A,B,C,D,E,F,G,H,I,J,K,T")
g.createVertices("S,A,B,C,T")

g.createEdges("S", "A", 2)
g.createEdges("A","B", 1)
g.createEdges("A,B","C",4)
g.createEdges("A","T",10)
g.createEdges("C","T",5)

'''
g.createEdges("S",   "A",   2 ) 
g.createEdges("A,E", "B,I", 20 ) 
g.createEdges("B,B", "C,F", 14 ) 
g.createEdges("D",   "E",   9 ) 
g.createEdges("E",   "F",   10 ) 
g.createEdges("F",   "G",   25 ) 
g.createEdges("H",   "I",   18 ) 
g.createEdges("I",   "J",   8 ) 
g.createEdges("J",   "T",   11 ) 
g.createEdges("S",   "D",   13 ) 
g.createEdges("A",   "E",   27) 
g.createEdges("C,D", "G,H", 15) 
g.createEdges("F",   "J",   12) 
g.createEdges("G",   "T",   7) 
'''

#print(g.vertexList)
#for v in g.vertexList:
#    print(v.relatedEdges)
#// Aditional edges for special cases testing
#g.createEdges("E", "J", 30, "beta")   
#g.createEdges("F", "T", 35, "beta")
#g.createEdges("J", "K", 5,  "beta")
#g.createEdges("C", "C", 16, "beta")  

p = g.findShortestPath("S","T")

while p.isValid():
	print(p.getVertexNames() + "( " + p.getWeight() + ")")
	p = g.FindNextShortestPath()
else:
    print("Not valid all")

