import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from stl import mesh

fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))

your_mesh = mesh.Mesh.from_file("../meshes/test.stl")
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors, color='red'))
scale = your_mesh.points.flatten()
ax.auto_scale_xyz(scale, scale, scale)
for points, normals in zip(your_mesh.points, your_mesh.normals):
    print(points)
    print(normals)
    print()
# plt.show()
