# Graph
```python
# create a graph with vertices 
graph = collections.defaultdict(list)
for (u, v, w) in times:
    graph[u].append((v, w))
```
## DFS
```py

def dfs(u, visited = set() ):

    # Mark the current node as visited
    visited.add(u)

    # Recur for all the vertices adjacent to this vertex
    for v in graph[u]:
        if v not in visited:
            dfs(v, visited)
```


## BFS
```py

def bfs(graph, start, end):
    # create BFS dequeue which has start vertex
    queue = [(start,[start])]
    visited = set()

    while queue:
        vertex, path = queue.pop(0)
        visited.add(vertex)

        for node in graph[vertex]:
            if node == end:
                return path + [end] # found shortest path
            else:
                if node not in visited:
                    visited.add(node)
                    queue.append((node, path + [node]))

    
    return -1 # no path found
```

## Dijkstra
```py
def dijkstra(graph, start):
    inf = float('inf')
    n = len(graph.keys())

    dist = {v: inf for v in graph}
    visited = set()
    # initial value of all vertices are inf

    # set start vertex as 0
    dist[start] = 0
    minHeap = [(0, start)]

    while minHeap:
        # get current min weight and current vertex
        weight, u = heapq.heappop(minHeap)
        if u in visited:
            continue
        visited.add(u)
        
        for v in graph[u]:
            if v not in visited and dist[v] > dist[u] + graph[u][v]:
                dist[v] = dist[u] + graph[u][v]
                heapq.heappush(minHeap, (dist[v], v))
    return dist
```