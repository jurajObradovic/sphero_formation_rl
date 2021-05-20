#!/usr/bin/env python3


import matplotlib.pyplot as plt
import numpy as np


initialDistances = [(3, "green"), (2,"blue"),  (1, "red")]
currentDistances = np.arange(4, 0.5, -0.05).tolist()
distanceRate = []
labels = []

color=["red", "green", ]

for initial in initialDistances:

	for current in currentDistances:
		if (current < initial[0]):
			distanceRate.append(2)
		else:
			distanceRate.append(2 ** (current / initial[0]))



	plt.plot(currentDistances, distanceRate, initial[1])
	labels.append("Početna udaljenost = {}".format(initial[0]))
	distanceRate = []



plt.plot(currentDistances, (np.ones(len(currentDistances)) * 2).tolist(), "k--")

plt.title("Fukcija nagrade udaljenosti za različite početne udaljenosti")
plt.xlabel("Trenutna udaljenost od cilja")
plt.ylabel("Nagrada za udaljenost")
plt.grid()
plt.legend(labels)
plt.show()

	




