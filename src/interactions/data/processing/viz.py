import pandas as pd
import pickle
import matplotlib.pyplot as plt

# unpickle the data in ../path.pkl
with open('../../../../path.pkl', 'rb') as f:
    pathf = pickle.load(f)

# flatten the path if it is a list of lists
path = []
for sublist in pathf:
    if isinstance(sublist, list):
        for item in sublist:
            path.append(item)
    else:
        path.append(sublist)

positions = []
orientations = []

for entry in path:
    position = entry.position
    orientation = entry.orientation
    positions.append([position.x, position.y, position.z])
    orientations.append([orientation.x, orientation.y, orientation.z, orientation.w])

# Create a DataFrame with specified column names
df = pd.DataFrame({
    'px': [pos[0] for pos in positions],
    'py': [pos[1] for pos in positions],
    'pz': [pos[2] for pos in positions],
    'ox': [ori[0] for ori in orientations],
    'oy': [ori[1] for ori in orientations],
    'oz': [ori[2] for ori in orientations],
    'ow': [ori[3] for ori in orientations]
})

# Print the resulting DataFrame
print(df)

# graph the df: px against py as scatter plot
fig, ax = plt.subplots()

# Extract position and quaternion components from the DataFrame
x = df['px'].to_numpy()
y = df['py'].to_numpy()
z = df['pz'].to_numpy()
ox = df['ox'].to_numpy()
oy = df['oy'].to_numpy()
oz = df['oz'].to_numpy()
ow = df['ow'].to_numpy()

# make scatter markers smallest possible
ax.scatter(x, y, marker='.', s=0.1)

ax.set_xlabel('x')
ax.set_ylabel('y')

center = [-0.135,0.650]
ax.scatter(center[0], center[1], marker='x', color='red')

plt.show()


fig = plt.figure(figsize=(9, 6))
ax = plt.axes(projection='3d')

import math

#ax.scatter3D(x[:math.floor(0.3*len(x))], y[:math.floor(0.3*len(x))], z[:math.floor(0.3*len(x))], color='red')
ax.scatter3D(x, y, z, color='red')

ax.set_title("3D scatterplot", pad=25, size=15)
ax.set_xlabel("X") 
ax.set_ylabel("Y") 
ax.set_zlabel("Z")

plt.show()