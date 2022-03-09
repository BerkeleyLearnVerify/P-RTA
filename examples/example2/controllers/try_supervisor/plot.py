import matplotlib.pyplot as plt

f = open("all_trusted_50_rhos.txt", "r")
lines = f.readlines()
f.close()
all_trusted_rhos = list(filter(lambda x: "Rho" in x, lines))
all_trusted_rhos = list(map(lambda x: float(x.split()[1]), all_trusted_rhos))

f = open("rta_50_rhos.txt", "r")
lines = f.readlines()
f.close()
rhos = list(filter(lambda x: "Rho" in x, lines))
rhos = list(map(lambda x: float(x.split()[1]), rhos))


print("rhos", ",", "all_trusted_rhos")
for i in range(len(rhos)):
	print(i + 1, ",", rhos[i], ",", all_trusted_rhos[i])

plt.figure(figsize=(10, 6))
plt.plot(rhos, label="The P-RTA Implementation")
plt.plot(all_trusted_rhos, label="The Baseline Implementation")

plt.grid(True, linestyle='--')

plt.xlabel('Number of Iterations')

plt.ylabel('Minimum Battery Level Observed in Simulation')

plt.xticks(range(0, 11, 10))

plt.yticks(range(0, 101, 10))

plt.legend(loc="lower right")

plt.show()