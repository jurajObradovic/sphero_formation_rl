#!/usr/bin/env python3


#agent.savePath+str(agent.loadEpisodeFrom)+'.json'
#/tmp/spheroModel/

import json
import matplotlib.pyplot as plt

finalEpisode = 2510;
scoreList = []
epsilonList =  []
memoryList = []
averageQList = []
timeList = []


episodeCount = []
for i in range(finalEpisode):
	if(i % 10 == 0 and i != 0 ):
		f = open('/tmp/spheroModel/' + str(i) + '.json')
		data = json.load(f)

		scoreList.append(data["score"])
		epsilonList.append(data["epsilon"])
		memoryList.append(data["memory"])
		averageQList.append(data["averageQ"])
		timeList.append(data["time"])

		episodeCount.append(i)
		f.close()


#Ploting data

fig, axs = plt.subplots(2, 2)
axs[0,0].plot(episodeCount, scoreList, "tab:blue")
axs[0,0].set_title("Reward")
axs[0,1].plot(episodeCount, epsilonList, "tab:green")
axs[0,1].set_title("Epsilon")
axs[1,0].plot(episodeCount, memoryList, "tab:purple")
axs[1,0].set_title("Memory")
axs[1,1].plot(episodeCount, averageQList,"tab:red")
axs[1,1].set_title("Average max Q-value")
plt.show()


#Saving data to file so we could later plot again

jsonForLater = {
	'score': scoreList,
	'epsilon': epsilonList,
	'memory': memoryList,
	'averageQ': averageQList,
	'time': timeList
}

d = open("/home/juraj/Desktop/Diplomski/leaderFollow4.json", "w")

json.dump(jsonForLater, d)

d.close()


