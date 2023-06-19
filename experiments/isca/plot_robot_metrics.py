#!/usr/bin/python3
import numpy as np
import os
import matplotlib.pyplot as plt
from math import ceil
import csv

from matplotlib.patches import Circle, RegularPolygon
from matplotlib.path import Path
from matplotlib.projections.polar import PolarAxes
from matplotlib.projections import register_projection
from matplotlib.spines import Spine
from matplotlib.transforms import Affine2D

from URDFParser import URDFParser
from FPGACodegen import FPGACodegen
from DesignSpaceExploration import DesignSpaceExploration

graph_metrics = [
        "# of links",
        "Longest chain",
        "Avg leaf depth",
        "Max descendents",
        "Leaf depth stddev"
]

graph_metrics_values = [
        [7,12,15,    12,15, 19],
        [7, 3, 7,     9, 9,  7],
        [7, 3, 5,     9, 9,  4],
        [7, 3, 7,    12,15,  7],
        [0, 0, 2.308, 0, 0,1.6],
]

mm_metrics = [
        "MM sparsity",
        "MM inverse sparsity"
]

mm_metrics_values = [
        [0,0.75,0.56,0.125,0.24,0.765],
        [0,0.75,0.56,    0,   0,0.765],
]

## plotting

robots = [
        "iiwa",
        "HyQ",
        "Baxter",
        "jaco-2",
        "jaco-3",
        "HyQ-Manip"
]

### bar chart for graph metrics

x = np.arange(len(graph_metrics))
width = 0.1

fig, ax = plt.subplots(figsize=(7,3))
graph_metrics_values = np.array(graph_metrics_values).T
rects_0 = ax.bar(x - 2.5*width, graph_metrics_values[0], width, label=robots[0])
rects_1 = ax.bar(x - 1.5*width, graph_metrics_values[1], width, label=robots[1])
rects_2 = ax.bar(x - 0.5*width, graph_metrics_values[2], width, label=robots[2])
rects_3 = ax.bar(x + 0.5*width, graph_metrics_values[3], width, label=robots[3])
rects_4 = ax.bar(x + 1.5*width, graph_metrics_values[4], width, label=robots[4])
rects_5 = ax.bar(x + 2.5*width, graph_metrics_values[5], width, label=robots[5])

handles = [
    rects_0, rects_1, rects_2, rects_3, rects_4, rects_5
]
labels = robots

ax.set_ylabel("Graph metric value")

ax.set_xticks(x)
ax.set_xticklabels(graph_metrics, fontsize=8)
ax.legend(handles, labels, ncol=2, loc="upper center", fontsize=8)

fig.tight_layout()

plt.savefig("out/robot_graph_metrics.pdf")
plt.close(fig)

### bar chart for mm metrics

x = np.arange(len(robots))
width = 0.3

fig, ax = plt.subplots(figsize=(5,3))
rects_0 = ax.bar(x - 0.5*width, mm_metrics_values[0], width, label=graph_metrics[0])
rects_1 = ax.bar(x + 0.5*width, mm_metrics_values[1], width, label=graph_metrics[1])

handles = [
    rects_0, rects_1
]
labels = mm_metrics

ax.set_ylabel("Matrix sparsity")

ax.set_xticks(x)
ax.set_xticklabels(robots)
ax.legend(handles, labels, loc="upper left", fontsize=8)

fig.tight_layout()

plt.savefig("out/robot_mm_metrics.pdf")
plt.close(fig)

### radar chart for graph metrics, didn't work out

#def radar_factory(num_vars, frame='circle'):
#    """
#    Create a radar chart with `num_vars` axes.
#
#    This function creates a RadarAxes projection and registers it.
#
#    Parameters
#    ----------
#    num_vars : int
#        Number of variables for radar chart.
#    frame : {'circle', 'polygon'}
#        Shape of frame surrounding axes.
#
#    """
#    # calculate evenly-spaced axis angles
#    theta = np.linspace(0, 2*np.pi, num_vars, endpoint=False)
#
#    class RadarTransform(PolarAxes.PolarTransform):
#
#        def transform_path_non_affine(self, path):
#            # Paths with non-unit interpolation steps correspond to gridlines,
#            # in which case we force interpolation (to defeat PolarTransform's
#            # autoconversion to circular arcs).
#            if path._interpolation_steps > 1:
#                path = path.interpolated(num_vars)
#            return Path(self.transform(path.vertices), path.codes)
#
#    class RadarAxes(PolarAxes):
#
#        name = 'radar'
#        PolarTransform = RadarTransform
#
#        def __init__(self, *args, **kwargs):
#            super().__init__(*args, **kwargs)
#            # rotate plot such that the first axis is at the top
#            self.set_theta_zero_location('N')
#
#        def fill(self, *args, closed=True, **kwargs):
#            """Override fill so that line is closed by default"""
#            return super().fill(closed=closed, *args, **kwargs)
#
#        def plot(self, *args, **kwargs):
#            """Override plot so that line is closed by default"""
#            lines = super().plot(*args, **kwargs)
#            for line in lines:
#                self._close_line(line)
#
#        def _close_line(self, line):
#            x, y = line.get_data()
#            # FIXME: markers at x[0], y[0] get doubled-up
#            if x[0] != x[-1]:
#                x = np.append(x, x[0])
#                y = np.append(y, y[0])
#                line.set_data(x, y)
#
#        def set_varlabels(self, labels):
#            self.set_thetagrids(np.degrees(theta), labels)
#
#        def _gen_axes_patch(self):
#            # The Axes patch must be centered at (0.5, 0.5) and of radius 0.5
#            # in axes coordinates.
#            if frame == 'circle':
#                return Circle((0.5, 0.5), 0.5)
#            elif frame == 'polygon':
#                return RegularPolygon((0.5, 0.5), num_vars,
#                                      radius=.5, edgecolor="k")
#            else:
#                raise ValueError("Unknown value for 'frame': %s" % frame)
#
#        def _gen_axes_spines(self):
#            if frame == 'circle':
#                return super()._gen_axes_spines()
#            elif frame == 'polygon':
#                # spine_type must be 'left'/'right'/'top'/'bottom'/'circle'.
#                spine = Spine(axes=self,
#                              spine_type='circle',
#                              path=Path.unit_regular_polygon(num_vars))
#                # unit_regular_polygon gives a polygon of radius 1 centered at
#                # (0, 0) but we want a polygon of radius 0.5 centered at (0.5,
#                # 0.5) in axes coordinates.
#                spine.set_transform(Affine2D().scale(.5).translate(.5, .5)
#                                    + self.transAxes)
#                return {'polar': spine}
#            else:
#                raise ValueError("Unknown value for 'frame': %s" % frame)
#
#    register_projection(RadarAxes)
#    return theta
#
#
#def example_data():
#    data = [
#        graph_metrics, np.array(metrics_values).T.tolist(),
#    ]
#    return data
#
#
#if __name__ == '__main__':
#    N = 5
#    theta = radar_factory(N, frame='polygon')
#
#    data = example_data()
#    spoke_labels = data[0]
#
#    fig, ax = plt.subplots(figsize=(5, 5),
#                            subplot_kw=dict(projection='radar'))
#    #fig.subplots_adjust(wspace=0.25, hspace=0.20, top=0.85, bottom=0.05)
#
#    colors = ['b', 'r', 'g', 'm', 'y', 'c']
#    # Plot the four cases from the example data on separate axes
#    #for ax, (title, case_data) in zip(axs.flat, data):
#    #    ax.set_rgrids([0.2, 0.4, 0.6, 0.8])
#    #    ax.set_title(title, weight='bold', size='medium', position=(0.5, 1.1),
#    #                 horizontalalignment='center', verticalalignment='center')
#    #    for d, color in zip(case_data, colors):
#    #        ax.plot(theta, d, color=color)
#    #        ax.fill(theta, d, facecolor=color, alpha=0.25, label='_nolegend_')
#    #    ax.set_varlabels(spoke_labels)
#
#    ax.set_rgrids(np.arange(0,20,5))
#    for d, color in zip(data[1], colors):
#        ax.plot(theta, d, color=color)
#        ax.fill(theta, d, facecolor=color, alpha=0.25, label='_nolegend_')
#    ax.set_varlabels(spoke_labels)
#
#    # add legend relative to top-left plot
#    labels = robots
#    legend = ax.legend(labels, loc=(0.7, .85),
#                              labelspacing=0.1, fontsize='small')
#
#    plt.savefig("out/robot_graph_metrics_radar.pdf")


