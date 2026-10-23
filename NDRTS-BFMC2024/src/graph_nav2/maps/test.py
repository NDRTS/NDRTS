import networkx as nx
import sys
import pprint
import pathlib

g = nx.read_graphml(
    "/home/ndrts/workspaces/NDRTS-BFMC2024/src/graph_nav2/maps/map.graphml")
print("node IDs in map.graphml:")
pprint.pprint(list(g.nodes())[:30])          # first 30, just to see format
print(f"\nTotal nodes: {g.number_of_nodes()}")
