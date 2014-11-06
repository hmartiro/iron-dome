"""

"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

if __name__ == '__main__':

    from ast import literal_eval

    with open('projectile_data.out', 'r') as f:
        raw_data = literal_eval(f.read())

    projectiles = set([u[0] for v in raw_data for u in v[1:]])

    data = {}
    for p in projectiles:
        data[p] = [(r[0],) + e[1] for r in raw_data for e in r[1:] if e[0] == p]
        print(p)
        print(data[p])
        #subset = filter(lambda x: p in [y[0] for y in x[1:]], raw_data)
        data[p] = np.array(data[p]).T
        #data[p] = np.array([(x[0],) + x[1][1] for x in subset]).T

    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    colors = ["#0000FF", "#00FF00", "#FF0000", "#00FFFF", "#FF00FF",
        "#FFFF00", "#555555", "#FFA500", "#000000", "#A52A2A",
        "#556B2F", "#8B0000", "#2F4F4F", "#FFC0CB", "#222222", "#AAAAAA",
        "#123456", "#654321"
    ]

    lines = [ax.plot(data[p][1], data[p][2], data[p][3],
        c=colors[(p-1) % len(colors)]) for p in data]
    #lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]

    y, z = np.mgrid[-0.5:0.5:30j, 0:2:30j]
    ax.plot_wireframe(0, y, z, color="g")

    proxies = [matplotlib.lines.Line2D([0],[0], linestyle="none",
        c=colors[(p-1) % len(colors)], marker = 'o') for p in projectiles]

    ax.legend(
        proxies,
        ['t0 = {}'.format(data[p][0][0]) for p in projectiles],
        numpoints=1
        )

    # Setting the axes properties
    ax.set_xlim3d([0.0, 4.0])
    ax.set_xlabel('X')

    ax.set_ylim3d([-1.0, 1.0])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0.0, 3.0])
    ax.set_zlabel('Z')

    ax.set_title('Projectiles, Raw Measurements')

    plt.show()
