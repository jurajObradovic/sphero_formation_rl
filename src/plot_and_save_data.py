#!/usr/bin/env python3


#agent.savePath+str(agent.loadEpisodeFrom)+'.json'
#/tmp/spheroModel/

import json
import matplotlib.pyplot as plt

finalEpisode = 6545;
scoreList = []
epsilonList =  []
memoryList = []
averageQList = []
timeList = []
numOfCrashes = []
numOfTargets = []


episodeCount = []
for i in range(finalEpisode):
	if(i % 10 == 0 and i != 0 and i != 930 ):
		f = open('/tmp/spheroModel/' + str(i) + '.json')
		data = json.load(f)

		scoreList.append(data["score"])
		epsilonList.append(data["epsilon"])
		memoryList.append(data["memory"])
		averageQList.append(data["averageQ"])
		timeList.append(data["time"])
		numOfCrashes.append(data["numOfCrashes"])
		numOfTargets.append(data["numOfTargets"])

		episodeCount.append(i)
		f.close()


#Ploting data

fig, axs = plt.subplots(2, 3)
axs[0,0].plot(episodeCount, scoreList, "tab:blue")
axs[0,0].set_title("Reward")
axs[0,1].plot(episodeCount, epsilonList, "tab:purple")
axs[0,1].set_title("Epsilon")
axs[1,0].plot(episodeCount, memoryList, "tab:brown")
axs[1,0].set_title("Memory")
axs[1,1].plot(episodeCount, averageQList,"tab:orange")
axs[1,1].set_title("Average max Q-value")
axs[1,2].plot(episodeCount, numOfCrashes,"tab:red")
axs[1,2].set_title("Number of crashes in 10 episodes")
axs[0,2].plot(episodeCount, numOfTargets,"tab:green")
axs[0,2].set_title("Number of times robot reached the target in 10 episodes")
plt.show()


#Saving data to file so we could later plot again

jsonForLater = {
	'score': scoreList,
	'epsilon': epsilonList,
	'memory': memoryList,
	'averageQ': averageQList,
	'time': timeList,
	'numOfCrashes': numOfCrashes,
	'numOfTargets': numOfTargets
}

d = open("/home/juraj/Desktop/Diplomski/leaderFollow4.json", "w")

json.dump(jsonForLater, d)

d.close()


