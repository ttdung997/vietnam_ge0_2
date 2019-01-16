import pandas as pd
import pickle
import matplotlib.pyplot as plt
from matplotlib import style
from datetime import datetime


pickle_in = open('clowflyP1.pickle','rb')
clowflyDf1 = pickle.load(pickle_in)

pickle_in = open('clowflyP2.pickle','rb')
clowflyDf2 = pickle.load(pickle_in)

pickle_in = open('clowflyP3.pickle','rb')
clowflyDf3 = pickle.load(pickle_in)


clowflyDf =  pd.concat([clowflyDf1, clowflyDf2, clowflyDf3])
clowflyDf =clowflyDf[(clowflyDf['Efficiency'] < 1 )]
clowflyDf =clowflyDf[(clowflyDf['Straightness'] < 1 )]
clowflyDf =clowflyDf[(clowflyDf['Efficiency'] > 0.1 )]
clowflyDf =clowflyDf[(clowflyDf['Straightness'] >0.1 )]

pickle_out = open('clowfly.pickle','wb')
pickle.dump(clowflyDf, pickle_out)
pickle_out.close() 

print(clowflyDf.describe())
print(clowflyDf.head())

# fig = plt.figure()
# ax1 = plt.subplot2grid((1,1), (0,0))

# plt.boxplot(clowflyDf['Efficiency'])
# plt.boxplot(clowflyDf['Straightness'])


# plt.title('Eficiency centrality and Straightness centrality)')

# plt.show()
Efficiency=clowflyDf['Efficiency']
hist = np.histogram(Efficiency, range=(np.amin(Efficiency),np.amax(Efficiency)),bins=len(Efficiency)/10)
hist = list(hist)
#hist[0] = hist[0]*100
#hist[0] = hist[0]
hist[1] = hist[1][:-1]

fig, ax = plt.subplots()
ax.set_axisbelow(True)

rects1 = ax.bar(hist[1], hist[0], 0.001, color='b')
ax.set_xlabel('Eficiency centrality')
ax.set_ylabel('Node')
plt.title('Eficiency centrality Distribution')

plt.show()

Straightness=clowflyDf['Straightness']
hist = np.histogram(Straightness, range=(np.amin(Straightness),np.amax(Straightness)),bins=len(Straightness)/10)
hist = list(hist)
#hist[0] = hist[0]*100
#hist[0] = hist[0]
hist[1] = hist[1][:-1]

fig, ax = plt.subplots()
ax.set_axisbelow(True)

rects1 = ax.bar(hist[1], hist[0], 0.001, color='b',edgecolor='k')
ax.set_xlabel('Straightness centrality')
ax.set_ylabel('Node')
plt.title('Straightness centrality Distribution')

plt.show()