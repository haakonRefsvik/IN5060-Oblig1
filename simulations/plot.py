from itertools import chain
import matplotlib.pyplot as plt
import numpy as np

from data import getCountrySideData, getVillageData, getCityData

# Map sizes
map_sizes = np.arange(10, 110, 10)
env = "Countryside"
data = getCountrySideData()
#data = getVillageData()
#data = getCityData()

dijkstra_cost = data[0]
dijkstra_time = data[1]
astar_cost = data[2]
astar_time = data[3]
jps_cost = data[4]
jps_time = data[5]
gbfs_cost = data[6]
gbfs_time = data[7]

from itertools import chain
import matplotlib.pyplot as plt
import numpy as np

# Colors for each algorithm
colors = ['lightblue', 'orange', 'lightgreen', 'pink']
labels = ['A*', 'Dijkstra', 'JPS', 'GBFS']

# ---------------------------------------------------------
# Box Plot – PATH COST
# ---------------------------------------------------------
plt.figure(figsize=(10,6))
box = plt.boxplot(
    [
    list(chain.from_iterable(astar_cost)),
    list(chain.from_iterable(dijkstra_cost)),
    list(chain.from_iterable(jps_cost)),
    list(chain.from_iterable(gbfs_cost))],
    labels=labels,
    patch_artist=True,
    showfliers=False  # Hides outliers
)

# Apply colors
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)

# Legend
for i in range(len(labels)):
    plt.plot([], c=colors[i], label=labels[i])
plt.legend()

plt.ylabel("Cost (in squares)")
plt.title(f"Distribution of Cost in {env}")
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()

# ---------------------------------------------------------
# Box Plot – COMPUTATION TIME
# ---------------------------------------------------------
plt.figure(figsize=(10,6))
box = plt.boxplot(
    [
     list(chain.from_iterable(astar_time)),
     list(chain.from_iterable(dijkstra_time)),
     list(chain.from_iterable(jps_time)),
     list(chain.from_iterable(gbfs_time)),
     ],
    labels=labels,
    patch_artist=True,
    showfliers=False  # Hides outliers
)

# Apply colors
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)

# Legend
for i in range(len(labels)):
    plt.plot([], c=colors[i], label=labels[i])
plt.legend()

plt.ylabel("Computation Time (s)")
plt.title(f"Distribution of Computation Time(s) in {env}")
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()
