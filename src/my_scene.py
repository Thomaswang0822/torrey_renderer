# This Python script follow the general automation idea of @BachLi
# It generates a hemisphere surface made of many small spheres
# Many calculation details are provided and/corrected by ChatGPT

import numpy as np

# Radius of big sphere
bigR = 3.0
# Radius of unit spheres (not 'unit' radius though)
unitR = 0.5

# Number of unit spheres to fill up the circumference of the big sphere
n = int(
    (2 * np.pi * bigR) / 
        (2 * unitR) 
)

# Generate positions of unit spheres on the surface of the big sphere
theta = np.linspace(0, np.pi/2, n//2)   # linespace(from, to, how_many)
phi = np.linspace(0, 2 * np.pi, n)
theta, phi = np.meshgrid(theta, phi)
theta = theta.flatten()
phi = phi.flatten()
x = bigR * np.sin(theta) * np.cos(phi)
y = bigR * np.sin(theta) * np.sin(phi)
z = bigR * np.cos(theta)

# Create a list of sphere objects
spheres = []
for i in range(len(x)):
    # xyz in normal math: right, forward (into sceen), up
    # xyz in graphic system (here): right, up, into screen
    center = [x[i], z[i], y[i]]
    spheres.append({'center': center, 'radius': unitR})

# Create a list of material objects
n_spheres = len(spheres)
print(f"Number of spheres: {n_spheres}")
colors = np.random.rand(3, n_spheres)
materials = []
for i in range(n_spheres):
    mt = 'Diffuse' if np.random.rand() > 0.5 else 'Mirror'
    color = colors[:, i]
    materials.append({'type': mt, 'color': color})

# Print the sphere and material vectors
print('std::vector<Sphere>{')
for i in range(n_spheres):
    center = spheres[i]['center']
    radius = spheres[i]['radius']
    print(f'\t{{Vector3{{{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}}}, {radius:.3f}, {i}}},')
print('},')
print('std::vector<Material>{')
for i in range(n_spheres):
    mt = materials[i]['type']
    color = materials[i]['color']
    print(f'\t{{MaterialType::{mt}, Vector3{{{color[0]:.3f}, {color[1]:.3f}, {color[2]:.3f}}}}},')
print('}')
