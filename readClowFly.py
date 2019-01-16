import pandas as pd
import pickle
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from matplotlib import style
from datetime import datetime


pickle_in = open('clowfly.pickle','rb')
clowflyDf = pickle.load(pickle_in)
print(clowflyDf.head())
G = nx.read_gexf('./R_VN_NHW_Inventory_1_connected_component.gexf')
for nodeIndex in G.nodes():
	G.node[nodeIndex]['Efficiency'] = 0
	G.node[nodeIndex]['Straightness'] = 0
for index, row in clowflyDf.iterrows():
	Efficiency=row[0]
	Straightness=row[1]
	G.node[str(int(row[2]))]['Efficiency'] = float(Efficiency)
	G.node[str(int(row[2]))]['Straightness'] = float(Straightness)


nx.write_gexf(G,'./R_VN_NHW_Inventory_1_connected_component_clowfly.gexf')