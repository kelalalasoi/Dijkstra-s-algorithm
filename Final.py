# -*- coding: utf-8 -*-
"""
Created on Wed Nov 29 04:48:17 2023

@author: Kelsie Nguyen
"""
#LIST FOR SHORTEN NAME OF TRAIN STATION (AS REFERENCE)
shorten_list = [
    ("Paddington", "P"),
    ("Notting Hill Gate", "NT"),
    ("Baker Street", "BS"),
    ("Bond Street", "BO"),
    ("South Kensington", "SK"),
    ("Green Park", "GP"),
    ("Victoria", "VI"),
    ("Oxford Circus", "OC"),
    ("Piccadilly Circus", "PC"),
    ("Westminster", "WE"),
    ("Warren Street", "WS"),
    ("Tottenham Court Road", "TC"),
    ("Leicester Square", "LS"),
    ("Charing Cross", "CC"),
    ("Embankment", "EM"),
    ("Waterloo", "WL"),
    ("Kings Cross", "KC"),
    ("Holborn", "HO"),
    ("Blackfriars", "BF"),
    ("Elephant and Castle", "EC"),
    ("Moorgate", "MO"),
    ("Old Street", "OS"),
    ("Bank", "BA"), 
    ("Liverpool Street", "LV"),
    ("Tower Hill", "TH"),
    ("London Bridge", "LB"),
    ("Aldgate East", "AE")
]

"------------------------------------------------------------------------------"

#OPTION 1: Define Algorithm using class()
class Graph:
    #define initial station using Python constructor __init__ and parameter self
    def __init__(self):
        
        #with the parameter self, create a dictionary of node to store further information
        self.nodes = {}

    #define the add_note to store 3 parameters, the key is the identification while neighbours (edge) are the dictionary of the graph that will be presented later
    def add_node(self, key, neighbours):
        
        #the key parameter will be updated with the neighbours
        self.nodes[key] = neighbours

    #define the Dijkstra's algorithm with parameters self, start (initial station), and finish (destination)
    def shortest_path(self, start, finish):
        
        #dictionary that stores distance to start node of a vertex
        distance = {}
        
        #dictionary that stores the visited node (visited station) after every iteration
        visited = {}
        
        #dictionary that stores the priority queue (PQ) value for each iteration
        queue = {}

        #innitialize the dictionary for each node and their edges
        for node in self.nodes:
            
            #using if to seperate the initial points and all other stations
            #with the starting point, make sure the distance to itself is equal 0
            if node == start:
                
                #distance from the start node is 0
                distance[node] = 0 
                
                #the queue now is still 0
                queue[node] = 0
                
            #if the node is not the initial point, we have other condition to make sure the algorithm will keep iterating
            else:
                
                #set unvisited nodes arc length to infinity (according the the algorithm)
                distance[node] = float('inf')
                queue[node] = float('inf')

        #run a loop that makes sure the node will be updated until the shortest path (sometimes the whole map) was all visited
        while queue:
            
            #after each iteration, use min () to record the smallest value of edge from the initial point
            lowest_key = min(queue, key=queue.get)
            
            #once the node with the minimum distance is identified, delete that node from the priority queue
            del queue[lowest_key]

            #there will be 2 situation after the every iteration: find the shortest way to the destination already or still not yet
            #thus, if after that iteration, the lowest_key was identified
            if lowest_key == finish:
                
                #then record the total distance from initial station to that destination
                total_distance = distance[finish]
                
                #the algorithm will record only the information of innitial point to that specific destination
                shortest_path = [finish]
                
                #by using an inside loop for previous condition
                while True:
                    
                    #temporary value is the dictionary of visited nodes after the lowest key met the condition (finished)
                    temp_val = visited[lowest_key]
                    
                    #append the temp_val to the shortest_path, for backtrackig the process
                    shortest_path.append(temp_val)
                    
                    #update the lowest_key with dictionary of visited nodes after lowest key met the condition (finished)
                    lowest_key = visited[lowest_key]
                    
                    #if the lowest_key has reached the initial node, then the loop is existed
                    if lowest_key == start:
                        break
                
                #if the loop exist, reverse the order of nodes (because we append the nodes previously)
                shortest_path.reverse()
                
                #then print the result
                print(f"From {start} to {finish}:")
                print(f"Total Distance: {total_distance}")
                print("Route:", " -> ".join(shortest_path)) #with each nodes are store respectively, use "->" to join them to make it look professional
            
            #explore neighbors of the current node released from the Priority Queue
            #due to the fact that the algorithm is so powerful in finding shortest path from all node to all others, not only for specific start and finish nodes
            else:
                
                #checks neighbors of the node released from Priority Queue
                for neighbour in self.nodes[lowest_key].keys():
                    
                    #alternative path will be added with the value of the neighbor to the distance of the previous node
                    alt_path = distance[lowest_key] + self.nodes[lowest_key][neighbour]
                    
                    #if the new path is shorter than the distance of the node in Priority Queue, then pq should be updated
                    #because not all the path that have edge with visited nodes are actually have the smallest value, we also have to examine the whole graph
                    if alt_path < distance[neighbour]:
                        
                        #change the distance
                        distance[neighbour] = alt_path  
                        
                        #update the visited dictionary as well
                        visited[neighbour] = lowest_key
                        
                        # now changes are made to Priority Queue
                        queue[neighbour] = alt_path


#in this step, I present the map (Graph) by using dictionary that contain node and edge
#there are 2 ways of presenting the given map: adjacency lists (using dictionaries) and adjacency matrics
#i chose list (node, edge) due to the folowing reason: 
#dictionaries allow for dynamic addition and removal of nodes and edges without needing to resize data structure
#dictionaries are well-suited for representing graphs with arbitrary structures and varying degrees of connectivity between nodes
#in a matrix representation, it is needed to allocate space for all possible edges, leading to wastage of memory for sparse graphs
g = Graph()

g.add_node("P", {"P": 0, "BS": 6, "NT": 4})
g.add_node("BS", {"BS": 0, "P": 6, "BO": 2, "KC": 7, "OC": 4})
g.add_node("NT", {"NT": 0, "P": 4, "BO": 7, "SK": 7})
g.add_node("BO", {"BO": 0, "BS": 2, "NT": 7, "OC": 1, "GP": 2})
g.add_node("GP", {"GP": 0, "BS": 2, "OC": 2, "SK": 7, "VI": 2, "PC": 1, "WE": 3})
g.add_node("SK", {"SK": 0, "GP": 7, "NT": 7, "VI": 4})
g.add_node("VI", {"VI": 0, "GP": 2, "SK": 4, "WE": 4})
g.add_node("OC", {"OC": 0, "BS": 4, "BO": 1, "GP": 2, "WS": 2, "TC": 2, "PC": 2})
g.add_node("PC", {"PC": 0, "OC": 2, "GP": 1, "LS": 2, "CC": 2})
g.add_node("WE", {"WE": 0, "VI": 4, "GP": 3, "EM": 2, "WL": 2})
g.add_node("WS", {"WS": 0, "OC": 2, "TC": 3, "KC": 3})
g.add_node("TC", {"TC": 0, "WS": 3, "OC": 2, "HO": 2, "LS": 1})
g.add_node("LS", {"LS": 0, "TC": 1, "PC": 2, "HO": 2, "CC": 2})
g.add_node("CC", {"CC": 0, "LS": 2, "PC": 2, "EM": 1})
g.add_node("EM", {"EM": 0, "CC": 1, "WE": 2, "WL": 2, "BF": 4})
g.add_node("WL", {"WL": 0, "EM": 2, "WE": 2, "LB": 3, "EC": 4})
g.add_node("EC", {"EC": 0, "WL": 4, "LB": 3})
g.add_node("HO", {"HO": 0, "TC": 2, "LS": 2, "KC": 4, "BA": 5})
g.add_node("KC", {"KC": 0, "BS": 7, "WS": 3, "HO": 4, "OS": 6, "MO": 6})
g.add_node("BF", {"BF": 0, "EM": 4, "BA": 4})
g.add_node("OS", {"OS": 0, "KC": 6, "MO": 1})
g.add_node("MO", {"MO": 0, "KC": 6, "OS": 1, "BA": 3, "LV": 2})
g.add_node("BA", {"BA": 0, "HO": 5, "BF": 4, "MO": 3,"LB": 2, "LV": 2, "TH": 2})
g.add_node("LB", {"LB": 0, "BA": 2, "WL": 3, "EC": 3})
g.add_node("LV", {"LV": 0, "MO": 2, "BA": 2, "TH": 6, "AE": 4})
g.add_node("TH", {"TH": 0, "LV": 6, "BA": 2, "AE": 2})
g.add_node("AE", {"AE": 0, "TH": 2, "LV": 4})

#code for testing
#1
g.shortest_path("OS", "PC")
#Result 1: 
    #From OS to PC:
    #Total Distance: 13
    #Route: OS -> KC -> WS -> OC -> PC

#2
g.shortest_path("PC", "OS")
#Result 2: 
    #From PC to OS:
    #Total Distance: 13
    #Route: PC -> OC -> WS -> KC -> OS

#3
g.shortest_path("GP", "AE")
#Result 3: 
    #From GP to AE:
    #Total Distance: 14
    #Route: GP -> PC -> LS -> HO -> BA -> TH -> AE

#4
g.shortest_path("KC", "LV")
#Result 4: 
    #From KC to LV:
    #Total Distance: 8
    #Route: KC -> MO -> LV

#5
g.shortest_path("TH", "SK")
#Result 5:
    #From TH to SK:
    #Total Distance: 17
    #Route: TH -> BA -> LB -> WL -> WE -> VI -> SK

#If the map need to be updated, modify the dictionary with updated information, or create a new one with all edges information

"------------------------------------------------------------------------------"

#OPTION 2: Define Algorithm using def()

#efine the Dijkstra's algorithm with parameters graph, start (initial station), and finish (destination)
def shortest_path(graph, start, finish):
 
  #initialise a set of visited stations, to store initial node and visited nodes by the algorithm
  #it is both suitable to use set and list, but using set will show the independece of the key in this task
  visited = set()
  
  #initialise a set of not visited stations, to store the location (key) that the algorithm has not yet been through
  not_visited = set(graph.keys())
  
  #define the distance of stations to store the edge between initial node to its closet nodes
  distances = {}
  
  #define the path to update the shortest path when the algorithm is trying to reach all the nearest location that has direct path
  path = {}
  
  #create a loop for the distance of your initial location to other, mark all the value as infinite so it can be update in the future
  #value initial node's edge to itself equal 0
  for neighbours in not_visited:
    distances[neighbours] = float("inf") #set distance to other not visited nodes' edge as infinity
    path[neighbours] = [start]
  distances[start] = 0  #set distance to itself (initial node) as 0

#Create a loop to run the algorithm to all other not visited nodes
  while not_visited:
    
    #after each iteration, use min () to record the smallest value of edge from the initial point
    lowest_key = min(not_visited, key=lambda neighbours: distances[neighbours])

    #update the edge of all neighbours nodes
    for neighbours, weight in graph[lowest_key].items():
      if neighbours in not_visited:
        old_distance = distances[neighbours]
        new_distance = distances[lowest_key] + weight

        #update the distance with the shorter path (if any) after exploring all other neighbours
        if new_distance < old_distance:
          distances[neighbours] = new_distance
          path[neighbours] = list(path[lowest_key]) + [neighbours]
    #mark all the node those were explored as visited
    not_visited.remove(lowest_key) 
    visited.add(lowest_key)

  return distances[finish], path[finish]


#test the graph, by transcript the map in requirement into a dictionary, that have all the distance of one initial point to its all direct path
graph = {
    "P": {"BS": 6, "NT": 4},
    "BS": {"P": 6, "BO": 2, "KC": 7, "OC": 4},
    "NT": {"P": 4, "BO": 7, "SK": 7},
    "BO": {"BS": 2, "NT": 7, "OC": 1, "GP": 2},
    "GP": {"BS": 2, "OC": 2, "SK": 7, "VI": 2, "PC": 1, "WE": 3},
    "SK": {"GP": 7, "NT": 7, "VI": 4},
    "VI": {"GP": 2, "SK": 4, "WE": 4},
    "OC": {"BS": 4, "BO": 1, "GP": 2, "WS": 2, "TC": 2, "PC": 2},
    "PC": {"OC": 2, "GP": 1, "LS": 2, "CC": 2},
    "WE": {"VI": 4, "GP": 3, "EM": 2, "WL": 2},
    "WS": {"OC": 2, "TC": 3, "KC": 3},
    "TC": {"WS": 3, "OC": 2, "HO": 2, "LS": 1},
    "LS": {"TC": 1, "PC": 2, "HO": 2, "CC": 2},
    "CC": {"LS": 2, "PC": 2, "EM": 1},
    "EM": {"CC": 1, "WE": 2, "WL": 2, "BF": 4},
    "WL": {"EM": 2, "WE": 2, "LB": 3, "EC": 4},
    "EC": {"WL": 4, "LB": 3},
    "HO": {"TC": 2, "LS": 2, "KC": 4, "BA": 5},
    "KC": {"BS": 7, "WS": 3, "HO": 4, "OS": 6, "MO": 6},
    "BF": {"EM": 4, "BA": 4},
    "OS": {"KC": 6, "MO": 1},
    "MO": {"KC": 6, "OS": 1, "BA": 3, "LV": 2},
    "BA": {"HO": 5, "BF": 4, "MO": 3, "LB": 2, "LV": 2, "TH": 2},
    "LB": {"BA": 2, "WL": 3, "EC": 3},
    "LV": {"MO": 2, "BA": 2, "TH": 6, "AE": 4},
    "TH": {"LV": 6, "BA": 2, "AE": 2},
    "AE": {"TH": 2, "LV": 4}
}

#test code
#make sure to identify the start and destination first
#1
start_node = "MO"
finish_node = "TC"
#Result 1: 
    #From MO to TC:
    #Total Distance: 10
    #Route: MO -> BA -> HO -> TC

#2
start_node = "TC"
finish_node = "MO"
#Result 2: 
    #From TC to MO:
    #Total Distance: 10
    #Route: TC -> HO -> BA -> MO

#3
start_node = "WS"
finish_node = "BA"
#Result 3: 
    #From WS to BA:
    #Total Distance: 9
    #Route: WS -> TC -> HO -> BA

#4
start_node = "LB"
finish_node = "BS"
#Result 4: 
    #From LB to BS:
    #Total Distance: 10
    #Route: LB -> WL -> WE -> GP -> BS

#5
start_node = "P"
finish_node = "BF"
#Result 5: 
    #From P to BF: 
    #Total Distance: 18 
    #Route: P -> BS -> BO -> OC -> PC -> CC -> EM -> BF

#assign the nodes to algorithm
distances, path = shortest_path(graph, start_node, finish_node)

#then run print for result
print(f"From {start_node} to {finish_node}:")
print(f"Total Distance: {distances}")
print("Route:", " -> ".join(path))