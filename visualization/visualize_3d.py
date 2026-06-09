import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pandas as pd


def plot_3d_boxes(df, container_size):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Boxes' colors
    colors = ['skyblue', 'salmon', 'lightgreen', 'plum', 'khaki', 'coral', 'lightgray']

    for i, row in df.iterrows():
        x, y, z = row['x'], row['y'], row['z']
        w, h, d = row['w'], row['h'], row['d']

        # Vertices of the parallelepiped
        vertices = [
            [x, y, z],
            [x + w, y, z],
            [x + w, y + h, z],
            [x, y + h, z],
            [x, y, z + d],
            [x + w, y, z + d],
            [x + w, y + h, z + d],
            [x, y + h, z + d]
        ]

        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[4], vertices[5], vertices[6], vertices[7]],
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[1], vertices[2], vertices[6], vertices[5]],
            [vertices[4], vertices[7], vertices[3], vertices[0]]
        ]

        box = Poly3DCollection(faces, alpha=0.5, linewidths=1, edgecolors='k')
        box.set_facecolor(colors[i % len(colors)])
        ax.add_collection3d(box)

        # signing the ID
        ax.text(x + w / 2, y + h / 2, z + d / 2, str(row['id']), color='k', ha='center', va='center', fontsize=16)

    # Container
    ax.set_xlim(0, container_size[0])
    ax.set_ylim(0, container_size[1])
    ax.set_zlim(0, container_size[2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Visualization of Box Placement')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    dataframe = pd.read_csv("solution_75.csv")
    # dataframe = pd.read_csv("exact_solution.csv")
    plot_3d_boxes(dataframe, [587, 233, 220])
