import matplotlib.pyplot as plt
import numpy as np

# generate points
#a = np.random.randn(1,3)[0,:]
#b = np.random.randn(1,3)[0,:]
#c = np.random.randn(1,3)[0,:]

a = np.array([0.5,0,0.8])
b = np.array([0.5,-0.4,0.4])
c = np.array([0.5,0.4,0.4])

# create plane vectors
ab = b - a
ac = c - a
bc = c - b 

# calculate the normal vector of the plane
normal = np.cross(ab, ac)
normal = normal / np.linalg.norm(normal)

# generate a random robot position
x = np.random.randn(1,3)[0,:]
#x = np.array([0.4,0.4,0.4])

# distance form point x to 0
x_0 = np.dot(x, n)
# distance form point a to 0
a_0 = np.dot(a, normal)

# move point x to the intersection with the plane
new_x = x - np.dot(x, normal) * normal + a_0 * normal

ax = new_x - a
bx = ax - ab
# calculate the area of the triangle that is formed by the plane
s = np.dot(0.5, np.linalg.norm(np.cross(ab, ac)))

# calculate the area of the triangles that are formed by each side of the triangle and the point x
s1 = np.dot(0.5, np.linalg.norm(np.cross(bc, bx)))
s2 = np.dot(0.5, np.linalg.norm(np.cross(ab, ax)))
s3 = np.dot(0.5, np.linalg.norm(np.cross(ac, ax)))
p = s1/s
q = s2/s
r = s3/s
print(p, q, p)
print(p + q + r)

# if point ist in triangle all three triangles should be equal to the area of the triangle
if round(p + q + r, 10)  == 1:
    print('Point is inside the triangle')

# plot the vectors
fig = plt.figure()
vectors = fig.add_subplot(111, projection='3d')
vectors.set_xlim([-2, 2])
vectors.set_ylim([-2, 2])
vectors.set_zlim([-2, 2])
vectors.quiver(0, 0, 0, a[0], a[1], a[2], color='purple')
vectors.quiver(0, 0, 0, x[0], x[1], x[2], color='grey')
vectors.quiver(0, 0, 0, new_x [0], new_x [1], new_x [2], color='black')
vectors.quiver(a[0], a[1], a[2], normal[0], normal[1], normal[2], color='b')
vectors.quiver(a[0], a[1], a[2], ab[0], ab[1], ab[2], color='r')
vectors.quiver(a[0], a[1], a[2], ac[0], ac[1], ac[2], color='r')
vectors.quiver(a[0], a[1], a[2], new_x [0], new_x [1], new_x [2], color='g')
vectors.quiver(b[0], b[1], b[2], bc[0], bc[1], bc[2], color='r', )
plt.show()