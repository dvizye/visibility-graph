import heapq as hq
from collections import defaultdict
from geo import Edge

def djikstra(graph, start, end):
    queue = [(0, start, None)]
    visited = defaultdict(dict)
    while len(queue) > 0:
        (cost, current, parent) = hq.heappop(queue)
        if current not in visited:
            visited[current] = parent
            if current == end:
                # return cost
                return (cost, backtrack(visited, start, end))
            for distance, n in graph[current]:
                hq.heappush(queue, (cost + distance, n, current))
    return (cost, None)

def backtrack(graph, start, goal):
    current = goal
    path = []
    while current != start:
        next = graph[current]
        path.append(Edge(current, next))
        current = next
    path.reverse()
    return path

if __name__ == '__main__':
    G = {'a': [(5, 'b'), (10, 'c')],
         'b': [(7, 'd')], 
         'c': [(3, 'd')]
         }
    print djikstra(G, 'a', 'd')
