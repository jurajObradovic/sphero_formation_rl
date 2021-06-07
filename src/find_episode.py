#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt

num = 27;

f = open('/home/juraj/Desktop/Diplomski/LeaderFollow' + str(num) + '/leaderFollow' + str(num) +  '.json')
data = json.load(f)

numOfCrashes = data["numOfCrashes"]
numOfTargets = data["numOfTargets"]

bestFiveEpisodes = []
count = 0;

worstBest = (numOfTargets[0], 0)

for i in range(len(numOfCrashes)):
	if(numOfCrashes[i] <= 20):
		if(count < 3):
			count += 1
			bestFiveEpisodes.append((numOfTargets[i], i))
			if(numOfTargets[i] <= worstBest[0]):
				worstBest = (numOfTargets[i], i)
		else:
			if(numOfTargets[i] >= worstBest[0]):
				bestFiveEpisodes = [(numOfTargets[i], i) if x[1] == worstBest[1] else x for x in bestFiveEpisodes]
				worstBest = bestFiveEpisodes[0]
				for j in range(len(bestFiveEpisodes)):
					if(bestFiveEpisodes[j] <= worstBest):
						worstBest = bestFiveEpisodes[j]





print(bestFiveEpisodes)

