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
from util import flatten, lmap

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
        n_clusters = len(set(clusters))

        colors = [cm.nipy_spectral(float(x) / n_clusters) for x in clusters]
        plt.scatter(states.T[0], states.T[1], c=colors, alpha=0.3, s=80, linewidths=0)
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
        edges, edge_labels, _, centroids_by_state, _, _ = state_machine
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

    def plot(self, state_machine, path, plot=False):
        edges, edge_labels, points_by_state, centroids_by_state, _, _ = state_machine
        if plot:
            fig = plt.figure(figsize=(10, 8))
            self.plot_clusters(points_by_state)
            self.plot_state_machine(state_machine)
            fig.savefig(path + '.png', bbox_inches='tight')

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

    def clusters_to_state_machine(self, clusters, initial_state, state_machine=None):
        if not clusters:
            rospy.logerr("Could not cluster")
            return

        edges, reverse_edges, edge_labels, timestamps = {}, {}, {}, defaultdict(list)
        states_by_clusters = defaultdict(list)
        add_edge, remove_edge = self.get_edge_adder_and_remover(edges, reverse_edges, edge_labels)

        clusters = list(
            map(lambda cluster: cluster.cluster, clusters))
        topic = next(iter(self.dicts_by_topic))
        for i, prev_cluster_label in enumerate(clusters[:-1]):
            cluster_label = clusters[i + 1]
            states_by_clusters[prev_cluster_label].append(i)
            add_edge(prev_cluster_label, [cluster_label], topic)
            timestamps[(prev_cluster_label, cluster_label)].append(
                self.dicts_by_topic[topic][i + 1]['timestamp'] - self.dicts_by_topic[topic][i]['timestamp'])
        states_by_clusters[cluster_label].append(i + 1)

        self.divide_sync_topic_clusters(clusters, edges, edge_labels, reverse_edges, states_by_clusters, remove_edge,
                                        add_edge)

        initial_cluster = self.get_initial_cluster(initial_state, states_by_clusters)
        centroids = self.get_centroids(states_by_clusters)
        return edges, edge_labels, states_by_clusters, centroids, timestamps, initial_cluster

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
                min_distance_state = \
                min(map(lambda state: (distance.euclidean(self.data[state], logical_centroid), state),
                        states_by_clusters[cluster]), key=lambda x: x[0])[1]
                centroids_by_clusters[cluster] = topics_data[min_distance_state]
            except Exception as e:
                rospy.logerr(e)
        return centroids_by_clusters
