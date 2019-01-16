# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import pandas as pd
import pickle
from osgeo import ogr, osr
import networkx as nx
import numpy as np
from haversine import haversine

#calculate length of StringLine
def calculateGeometryLength(pointList, sourceSRS, destSRS):
    line = ogr.Geometry(ogr.wkbLineString)
    transform = osr.CoordinateTransformation(sourceSRS,destSRS)
    for point in pointList:
        line.AddPoint(point[0],point[1])
    line.Transform(transform)
    return line.Length()

# target srs for road length computation
target_srs = osr.SpatialReference()
target_srs.ImportFromProj4('+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs ')

highwayFileName = './roaddata/R_VN_NHW_Inventory.shp'
dataSource = ogr.Open(highwayFileName)
layer = dataSource.GetLayer(0)
source_srs = layer.GetSpatialRef()
featureCount = layer.GetFeatureCount()
print('featureCount: ', featureCount)

# get attribute list
attributeList = []
layerDefinition = layer.GetLayerDefn()
for i in range(layerDefinition.GetFieldCount()):
    fieldName =  layerDefinition.GetFieldDefn(i).GetName()
    attributeList.append(fieldName)

G = nx.Graph()
nodeList = []
i = 0
for feature in layer:
    geometry = feature.geometry()
    feature_length = geometry.Length()
    geometry.TransformTo(target_srs)
    pointCount = geometry.GetPointCount()
    pointList = geometry.GetPoints()
    pointList = np.around(np.array(pointList),decimals=10)
    pointList =  list(map(tuple, pointList))
    pointList = [tuple(map(float, point)) for point in pointList]
    ### first point ###########################################################
    firstPoint = pointList[0]
    if not firstPoint in nodeList:
        nodeList.append(firstPoint)
        G.add_node(i, lng=firstPoint[0], lat=firstPoint[1])
        firstNodeID = i
        i = i + 1
    else:
        for nodeidx in G.nodes_iter():
            if G.node[nodeidx]['lng'] == float(firstPoint[0]) and G.node[nodeidx]['lat'] == float(firstPoint[1]):
                firstNodeID = nodeidx
    ### last point ############################################################
    lastPoint = pointList[-1]
    if not lastPoint in nodeList:
        nodeList.append(lastPoint)
        G.add_node(i, lng=lastPoint[0], lat=lastPoint[1])
        lastNodeID = i
        i = i + 1
    else:
        for nodeidx in G.nodes_iter():
            if G.node[nodeidx]['lng'] == float(lastPoint[0]) and G.node[nodeidx]['lat'] == float(lastPoint[1]):
                lastNodeID = nodeidx
    # create middle list
    middlePointList = pointList[1:-1]
    ### add edge between points ###############################################
    G.add_edge(lastNodeID, firstNodeID)
    for attribute in attributeList:
        G[lastNodeID][firstNodeID][attribute] = feature.GetField(attribute) if feature.GetField(attribute) is not None else ''
        G[lastNodeID][firstNodeID]['length'] = feature_length
        # add middle list to edge attribute
        G[lastNodeID][firstNodeID]['middle'] = middlePointList[::-1]

    ### intersect processing ##################################################
    for edge in G.edges():
        headID = edge[0]
        tailID = edge[1]
        attributeDict = G[headID][tailID]
        middle = attributeDict['middle']
        if firstPoint in middle:
            if headID == firstNodeID or firstNodeID == tailID:
                continue
            indexFirstPoint = middle.index(firstPoint)
            # copy attributes
            attributeDictPart1 = attributeDict.copy()
            attributeDictPart2 = attributeDict.copy()
            # recalculate middle
            attributeDictPart1['middle'] = middle[0:indexFirstPoint]
            attributeDictPart2['middle'] = middle[indexFirstPoint+1:]
            # recalucate length
            roadPart1 = [(G.node[headID]['lng'],G.node[headID]['lat'])]
            roadPart1.extend(middle[0:indexFirstPoint+1])
            roadPart2 = middle[indexFirstPoint:]
            roadPart2.append((G.node[tailID]['lng'],G.node[tailID]['lat']))
            attributeDictPart1['length'] = calculateGeometryLength(roadPart1,source_srs,target_srs)
            attributeDictPart2['length'] = calculateGeometryLength(roadPart2,source_srs,target_srs)
            G.remove_edge(headID, tailID)
            G.add_edge(headID, firstNodeID, attr_dict=attributeDictPart1)
            G.add_edge(firstNodeID, tailID, attr_dict=attributeDictPart2)
        elif lastPoint in middle:
            if headID == lastNodeID or lastNodeID == tailID:
                continue
            indexLastPoint = middle.index(lastPoint)
            # copy attributes
            attributeDictPart1 = attributeDict.copy()
            attributeDictPart2 = attributeDict.copy()
            # recalculate middle
            attributeDictPart1['middle'] = middle[0:indexLastPoint]
            attributeDictPart2['middle'] = middle[indexLastPoint+1:]
            # recalculate length
            roadPart1 = [(G.node[headID]['lng'],G.node[headID]['lat'])]
            roadPart1.extend(middle[0:indexLastPoint+1])
            roadPart2 = middle[indexLastPoint:]
            roadPart2.append((G.node[tailID]['lng'],G.node[tailID]['lat']))
            attributeDictPart1['length'] = calculateGeometryLength(roadPart1,source_srs,target_srs)
            attributeDictPart2['length'] = calculateGeometryLength(roadPart2,source_srs,target_srs)
            G.remove_edge(headID, tailID)
            G.add_edge(headID, lastNodeID, attr_dict=attributeDictPart1)
            G.add_edge(lastNodeID, tailID, attr_dict=attributeDictPart2)


###test khoang cach


# print(G[0][2]['length'])
# node1 = (G.node[0]['lat'],G.node[0]['lng'])
# node2 = (G.node[2]['lat'],G.node[2]['lng'])
# longDistance=haversine(node1, node2)
# print("khoang cach",longDistance)
### remove middle properties ##################################################
for edge in G.edges_iter():
    G[edge[0]][edge[1]].pop('middle')
### check if 2 node same lat long #############################################
lat = G.node[0]['lat']
lng = G.node[0]['lng']
sameCount = -1
for i in G.nodes_iter():
    if G.node[i]['lat'] == lat and G.node[i]['lng'] == lng:
        sameCount += 1
    else:
        lat = G.node[i]['lat']
        lng = G.node[i]['lng']
print('same location Count: ',sameCount)
nodelength=len(G.nodes())
nodeEfficiencyList = []
nodeStraightnessList = []
nodelist = []
ShortPathLength = nx.all_pairs_dijkstra_path_length(G,weight = 'length')
for node1 in G.nodes():
 Ce=0
 ClowFlyStraight=0;
 DistanceStraight =0
 if node1 < 2000: continue
 for node2 in G.nodes():
   if node1 == node2: continue
   if not nx.has_path(G,node1,node2): continue
   geo1=(G.node[node1]['lat'],G.node[node1]['lng'])
   geo2=(G.node[node2]['lat'],G.node[node2]['lng'])
   if geo1 == geo2 : continue
   ClowFlyLength=haversine(geo1, geo2)*1000
   ClowFlyStraight = ClowFlyStraight +1/ClowFlyLength
   DistanceStraight = DistanceStraight +1/ShortPathLength[node1][node2]
   Ce=Ce+ClowFlyLength/ShortPathLength[node1][node2]
   # print("tu ", node1 ," den node ",node2,"ta co: ")
   # print("chim bay: ",ClowFlyLength)
   # print("duong bo 1: ",nx.shortest_path_length(G,node1,node2,weight = 'length'))
 Ce= Ce/nodelength
 Cs= DistanceStraight/ClowFlyStraight
 
 print("Node: ",node1)
 print("efficiency centrality: ",Ce)
 print("straightness centrality: ",Cs)
 nodeEfficiencyList.append(Ce)
 nodeStraightnessList.append(Cs)
 nodelist.append(node1)




data = {     'node'     : nodelist,
             'Efficiency': nodeEfficiencyList,
             'Straightness':nodeStraightnessList
             }

ResultDf = pd.DataFrame(data)

pickle_out = open('clowflyP3.pickle','wb')
pickle.dump(ResultDf, pickle_out)
pickle_out.close() 
   

# ### check for self loop in result graph #######################################
# self_loop_count = 0
# for node in G.nodes_iter():
#     if node in G.neighbors(node):
#         self_loop_count += 1
#         print(node, G.neighbors(node))
# print('self_loop_count: ', self_loop_count)
### remove little connected components due to wrong input data ################
# connected_components = list(nx.connected_component_subgraphs(G))
# G2 = connected_components[0]
### write graph to file #######################################################
#nx.write_gexf(G2,'./R_VN_NHW_Inventory_1_connected_component.gexf')
#nx.write_graphml(G2,'./R_VN_NHW_Inventory_1_connected_component.graphml')
layer = None
dataSource = None
