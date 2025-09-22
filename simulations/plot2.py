import numpy as np
import matplotlib.pyplot as plt
from data import getCountrySideData, getVillageData, getCityData
# Assume these functions return lists in the format you provided
data_countryside = getCountrySideData()
data_village = getVillageData()
data_city = getCityData()

all_data = [data_countryside, data_village, data_city]
algorithms = ["Dijkstra", "A*", "JPS", "GBFS"]

# Compute total cost (cost + time) per run, then average across runs and environments
totals = []

for data in all_data:
    # Sum cost + time element-wise for each run
    dijkstra_total = np.array(data[0]) + np.array(data[1])
    astar_total    = np.array(data[2]) + np.array(data[3])
    jps_total      = np.array(data[4]) + np.array(data[5])
    gbfs_total     = np.array(data[6]) + np.array(data[7])
    
    # Average across runs for this environment
    totals.append([
        np.mean(dijkstra_total),
        np.mean(astar_total),
        np.mean(jps_total),
        np.mean(gbfs_total)
    ])

totals = np.array(totals)  # shape: (3 environments, 4 algorithms)

# Average across environments
avg_totals = np.mean(totals, axis=0)

# Plot
plt.figure(figsize=(8,5))
plt.bar(algorithms, avg_totals, color=['orange', 'lightblue', 'lightgreen', 'pink'])
plt.ylabel("Average Total Cost (Cost + Time)")
plt.title("Average Algorithm Performance Across All Environments")
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()