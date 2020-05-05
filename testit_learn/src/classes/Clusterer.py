import math
import warnings
from collections import defaultdict
from copy import deepcopy
from itertools import count, cycle

from scipy.spatial import distance
from sklearn.cluster import MiniBatchKMeans
from sklearn.exceptions import ConvergenceWarning
from sklearn.metrics import silhouette_score, silhouette_samples
from sklearn.neighbors import NearestCentroid
from util import flatten

import rospy
import seaborn as sns
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt


class Clusterer:
    def __init__(self, data, dicts_by_topic, reduction):
        self.data = data
        self.dicts_by_topic = dicts_by_topic
        self.cluster = MiniBatchKMeans
        self.score = lambda clusters: silhouette_score(self.data, labels=clusters.labels_)
        self.initial_score = float('-inf')
        self.reduction_min = reduction.get('min', 2)
        self.reduction_max = reduction.get('max', len(data) / 2)
        self.better = lambda x, y: x > y
        self.clusters = None

    def get_clusters(self):
        best_evaluation = self.initial_score
        best_clusters = None

        n_clusters_min = int(len(self.data) / self.reduction_max)
        n_clusters_max = int(len(self.data) / self.reduction_min)
        with warnings.catch_warnings():
            warnings.filterwarnings('error', category=ConvergenceWarning)
            warnings.filterwarnings('error', category=RuntimeWarning)
            for n_clusters in range(n_clusters_min, n_clusters_max + 1):
                try:
                    clusters = self.cluster(n_clusters=n_clusters).fit(self.data)
                    evaluation = self.score(clusters)

                    if self.better(evaluation, best_evaluation):
                        best_evaluation = evaluation
                        best_clusters = clusters
                except ConvergenceWarning:
                    break
                except Exception as e:
                    rospy.logwarn(e)
        self.clusters = clusters
        return best_clusters

    def plot_clusters(self, states_by_clusters):
        sns.set_context('paper')
        sns.set_color_codes()

        clusters = []
        states = []
        for cluster in states_by_clusters:
            for state in states_by_clusters[cluster]:
                states.append(self.data[state])
                clusters.append(cluster)
        clusters = np.array(clusters)
        states = np.array(states)

        palette = sns.color_palette('bright', np.unique(clusters).max() + 1)
        colors = [palette[x] if x >= 0 else (0.0, 0.0, 0.0) for x in clusters]
        plt.scatter(states.T[0], states.T[1], c=colors, alpha=0.25, s=80, linewidths=0)
        # plt.title(self.cluster.__name__, fontsize=14)

    def plot_triangle_arrow(self, x, y, d, label, color):
        arrows = []
        arrows.append(plt.arrow(x, y, d, d, label=label, color=color))
        arrows.append(plt.arrow(x + d, y + d, -2 * d, 0, label=label, color=color))
        arrows.append(plt.arrow(x - d, y + d, d, -d, head_width=0.5, head_length=0.5, overhang=0,
                                length_includes_head=True, label=label, color=color))
        return arrows

    def add_to_plot_legend(self, label, arrow, plot_legend):
        arrows, labels = plot_legend
        if label in labels:
            return
        labels.append(label)
        arrows.append(arrow)

    def plot_state_machine(self, state_machine):
        edges, edge_labels, _, centroids_by_state, _ = state_machine
        centroids_by_state = deepcopy(centroids_by_state)
        colors = cycle(['blue', 'red', 'green', 'orange', 'purple'])
        arrow_colors = defaultdict(lambda: next(colors))
        plot_legend = ([], [])
        for state in edges:
            for next_state in edges[state]:
                x1, y1 = flatten(centroids_by_state[state])[0:2]
                x2, y2 = flatten(centroids_by_state[next_state])[0:2]
                dx = x2 - x1
                dy = y2 - y1
                label = edge_labels[(state, next_state)]
                color = arrow_colors[label]

                if dx == 0 and dy == 0:
                    triangle_arrows = self.plot_triangle_arrow(x1, y1, 3, label, color)
                    self.add_to_plot_legend(label, triangle_arrows.pop(), plot_legend)
                else:
                    arrow = plt.arrow(x1, y1, dx, dy, head_width=0.5, head_length=0.5, overhang=0,
                                      length_includes_head=True, label=label, color=color)
                    self.add_to_plot_legend(label, arrow, plot_legend)
        plt.legend(*plot_legend, fontsize=12)

    def plot_silhouette(self, data, clusters, path):
        """
            Taken from https://scikit-learn.org/stable/auto_examples/cluster/plot_kmeans_silhouette_analysis.html
        """
        cluster_labels = list(map(lambda cluster: cluster.cluster, clusters))
        fig, (ax1, ax2) = plt.subplots(1, 2)
        fig.set_size_inches(18, 7)
        ax1.set_xlim([-0.1, 1])
        n_clusters = len(set(cluster_labels))
        ax1.set_ylim([0, len(data) + (n_clusters + 1) * 10])

        silhouette_avg = silhouette_score(data, cluster_labels)
        sample_silhouette_values = silhouette_samples(data, cluster_labels)

        y_lower = 10
        for i in range(n_clusters):
            ith_cluster_silhouette_values = \
                sample_silhouette_values[cluster_labels == i]
            ith_cluster_silhouette_values.sort()
            size_cluster_i = ith_cluster_silhouette_values.shape[0]
            y_upper = y_lower + size_cluster_i
            color = cm.nipy_spectral(float(i) / n_clusters)
            ax1.fill_betweenx(np.arange(y_lower, y_upper),
                              0, ith_cluster_silhouette_values,
                              facecolor=color, edgecolor=color, alpha=0.7)
            ax1.text(-0.05, y_lower + 0.5 * size_cluster_i, str(i))
            y_lower = y_upper + 10

        ax1.set_title("The silhouette plot for the various clusters.")
        ax1.set_xlabel("The silhouette coefficient values")
        ax1.set_ylabel("Cluster label")
        ax1.axvline(x=silhouette_avg, color="red", linestyle="--")
        ax1.set_yticks([])  # Clear the yaxis labels / ticks
        ax1.set_xticks([-0.1, 0, 0.2, 0.4, 0.6, 0.8, 1])
        colors = cm.nipy_spectral(cluster_labels.astype(float) / n_clusters)
        ax2.scatter(data[:, 0], data[:, 1], marker='.', s=30, lw=0, alpha=0.7,
                    c=colors, edgecolor='k')

        ax2.set_title("The visualization of the clustered data.")
        ax2.set_xlabel("Feature space for the 1st feature")
        ax2.set_ylabel("Feature space for the 2nd feature")

        plt.suptitle(("Silhouette analysis for KMeans clustering on sample data "
                      "with n_clusters = %d" % n_clusters),
                     fontsize=14, fontweight='bold')

        plt.savefig(path + '-silhouette.png')

    def plot(self, state_machine, data, clusters, path, plot=False):
        edges, edge_labels, points_by_state, centroids_by_state, _ = state_machine
        if plot:
            fig = plt.figure(figsize=(10, 8))
            self.plot_clusters(points_by_state)
            # self.plot_state_machine(state_machine)
            fig.savefig(path + '.png', bbox_inches='tight')
            self.plot_silhouette(data, clusters, path)

    def get_edge_adder_and_remover(self, edges, reverse_edges, edge_labels):
        def add_edge(from_node, to_nodes, label):
            to_nodes = filter(lambda node: node != from_node, to_nodes)
            if from_node in edges:
                edges[from_node].update(to_nodes)
            else:
                edges[from_node] = set(to_nodes)

            for to_node in to_nodes:
                reverse_edges[to_node] = from_node
                edge_labels[(from_node, to_node)] = label

        def remove_edge(from_node, to_nodes):
            new_to_nodes = set()
            for node in edges[from_node]:
                if node not in to_nodes:
                    new_to_nodes.add(node)
            edges[from_node] = new_to_nodes

        return add_edge, remove_edge

    def get_initial_cluster(self, initial_state, states_by_clusters):
        initial_cluster = None
        min_dist = float('inf')
        for cluster in states_by_clusters:
            states = list(map(lambda state: list(self.data[state])[:len(initial_state)], states_by_clusters[cluster]))
            dist = 0
            for state in states:
                dist += math.sqrt(
                    sum(map(lambda coords: (float(coords[1]) - float(coords[0])) ** 2, zip(initial_state, state))))
            dist /= len(states)
            if dist < min_dist:
                min_dist = dist
                initial_cluster = cluster
        return initial_cluster

    def divide_sync_topic_clusters(self, clusters, edges, edge_labels, reverse_edges, states_by_clusters, remove_edge,
                                   add_edge):
        cluster_counter = count(max(clusters) + 1)
        for topic in list(self.dicts_by_topic.keys())[1:]:
            dicts = self.dicts_by_topic[topic]
            for i, attributes_dict in enumerate(dicts):
                if not attributes_dict['success']:
                    continue
                cluster = clusters[i]
                all_successful = all(map(lambda state: dicts[state]['success'], states_by_clusters[cluster]))

                new_cluster = next(cluster_counter)
                next_clusters = edges[cluster]
                prev_topic = edge_labels[(reverse_edges[cluster], cluster)]

                remove_edge(cluster, next_clusters)
                add_edge(cluster, [new_cluster], topic)
                add_edge(new_cluster, next_clusters, prev_topic)

                if all_successful:
                    states_by_clusters[new_cluster] = states_by_clusters[cluster]
                else:
                    states_by_clusters[cluster].remove(i)
                    states_by_clusters[new_cluster].append(i)

    def clusters_to_state_machine(self, clusters, initial_state, get_labels=lambda clusters: clusters.labels_):
        if not clusters:
            rospy.logerr("Could not cluster")
            return

        edges, reverse_edges, edge_labels = {}, {}, {}
        states_by_clusters = defaultdict(list)
        add_edge, remove_edge = self.get_edge_adder_and_remover(edges, reverse_edges, edge_labels)

        clusters = get_labels(clusters)
        topic = next(iter(self.dicts_by_topic))
        for i, prev_cluster_label in enumerate(clusters[:-1]):
            cluster_label = clusters[i + 1]
            states_by_clusters[prev_cluster_label].append(i)
            add_edge(prev_cluster_label, [cluster_label], topic)
        states_by_clusters[cluster_label].append(i + 1)

        self.divide_sync_topic_clusters(clusters, edges, edge_labels, reverse_edges, states_by_clusters, remove_edge,
                                        add_edge)

        initial_cluster = self.get_initial_cluster(initial_state, states_by_clusters)
        centroids = self.get_centroids(states_by_clusters)
        return edges, edge_labels, states_by_clusters, centroids, initial_cluster

    def get_centroids(self, states_by_clusters):
        centroids_by_clusters = {}

        cluster_labels = []
        cluster_data = []
        for cluster in states_by_clusters:
            for state in states_by_clusters[cluster]:
                cluster_data.append(self.data[state])
                cluster_labels.append(cluster)

        topics_data = [list() for _ in self.dicts_by_topic[next(iter(self.dicts_by_topic))]]
        for topic in self.dicts_by_topic:
            for i, dict_by_topic in enumerate(self.dicts_by_topic[topic]):
                topics_data[i].append(dict_by_topic['attributes'])
        centroid_finder = NearestCentroid().fit(cluster_data, cluster_labels)
        for cluster in cluster_labels:
            try:
                logical_centroid = centroid_finder.centroids_[cluster]
                centroid = topics_data[
                    min(map(lambda state: (distance.euclidean(self.data[state], logical_centroid), state),
                            states_by_clusters[cluster]), key=lambda x: x[0])[1]
                ]
                centroids_by_clusters[cluster] = centroid
            except Exception as e:
                rospy.logerr(e)
        return centroids_by_clusters
