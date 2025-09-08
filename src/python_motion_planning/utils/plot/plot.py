"""
Optimized Plot tools
- Batched plotting for expanded nodes to avoid per-node scatter calls
- Optional streaming (animated) update using chunked updates
- Reuses single PathCollection for 2D and single Collection for 3D to update offsets
@author: huiming zhou (optimized)
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from ..environment.env import Env, Grid, Map, Node

squaresize = 50
squaresize_explored = 5

class Plot:
    def __init__(self, start, goal, env: Env):
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        self.env = env
        self.fig = plt.figure("planning")
        # Use 3D axes if z_range is present
        if hasattr(env, 'z_range') and env.z_range is not None:
            # 3D
            from mpl_toolkits.mplot3d import Axes3D
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_box_aspect((self.env.x_range, self.env.y_range, self.env.z_range))  # X:Y:Z scaling
            self.is3d = True
        else:
            self.ax = self.fig.add_subplot()
            self.is3d = False

        # For optimized expand plotting: keep a reference to the scatter collections
        self._expand_scatter_2d = None
        self._expand_scatter_3d = None
        # Buffer for points already plotted (so we can append without replotting everything)
        self._expand_points = []

    def animation(self, path: list, name: str, cost: float = None, expand: list = None, history_pose: list = None,
                  predict_path: list = None, lookahead_pts: list = None, cost_curve: list = None,
                  ellipse: np.ndarray = None, expand_animate: bool = False, expand_subsample: int = 1) -> None:
        """
        expand_animate: if True, use chunked updates to animate expansion (slower but incremental)
        expand_subsample: sample factor for expanded nodes (1 -> all, 10 -> 1/10th)
        """
        name = name + "\ncost: " + str(cost) if cost else name
        self.plotEnv(name)
        if expand is not None:
            # optimized plotting
            self.plotExpand(expand, animate=expand_animate, subsample=expand_subsample)
        if history_pose is not None:
            self.plotHistoryPose(history_pose, predict_path, lookahead_pts)
        if path is not None:
            self.plotPath(path)

        if cost_curve:
            plt.figure("cost curve")
            self.plotCostCurve(cost_curve, name)

        if ellipse is not None:
            self.plotEllipse(ellipse)

        plt.show()

    def plotEnv(self, name: str) -> None:
        '''
        Plot environment with static obstacles.
        '''
        if self.is3d:
            self.ax.scatter(self.start.x, self.start.y, self.start.z, marker="s", color="#ff0000")
            self.ax.scatter(self.goal.x, self.goal.y, self.goal.z, marker="s", color="#1155cc")
            if isinstance(self.env, Grid):
                # Separate boundary walls from internal obstacles
                boundary_obs = []
                internal_obs = []

                for obs in self.env.obstacles:
                    x, y, z = obs
                    # Check if obstacle is on boundary
                    is_boundary = (x == 0 or x == self.env.x_range - 1 or 
                                 y == 0 or y == self.env.y_range - 1 or 
                                 z == 0 or z == self.env.z_range - 1)
                    if is_boundary:
                        boundary_obs.append(obs)
                    else:
                        internal_obs.append(obs)

                # Plot only selected boundary walls (remove top and front walls for better visibility)
                if boundary_obs:
                    visible_boundary = []
                    for obs in boundary_obs:
                        x, y, z = obs
                        # Skip top wall (z = max) and front wall (y = 0) for better visibility
                        if not (z == self.env.z_range - 1 or y == 0):
                            visible_boundary.append(obs)

                    if visible_boundary:
                        boundary_x = [x[0] for x in visible_boundary]
                        boundary_y = [x[1] for x in visible_boundary]
                        boundary_z = [x[2] for x in visible_boundary]
                        self.ax.scatter(boundary_x, boundary_y, boundary_z, c="lightgray", marker=".", s=squaresize, alpha=0)

                # Plot internal obstacles as solid
                if internal_obs:
                    internal_x = [x[0] for x in internal_obs]
                    internal_y = [x[1] for x in internal_obs]
                    internal_z = [x[2] for x in internal_obs]
                    self.ax.scatter(internal_x, internal_y, internal_z, c="black", marker="s", s=squaresize)
            # Map 3D visualization can be added here if needed
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
        else:
            plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000")
            plt.plot(self.goal.x, self.goal.y, marker="s", color="#1155cc")
            if isinstance(self.env, Grid):
                obs_x = [x[0] for x in self.env.obstacles]
                obs_y = [x[1] for x in self.env.obstacles]
                plt.plot(obs_x, obs_y, "sk")
            if isinstance(self.env, Map):
                ax = self.fig.add_subplot()
                for (ox, oy, w, h) in self.env.boundary:
                    ax.add_patch(patches.Rectangle(
                            (ox, oy), w, h,
                            edgecolor='black',
                            facecolor='black',
                            fill=True
                        )
                    )
                for (ox, oy, w, h) in self.env.obs_rect:
                    ax.add_patch(patches.Rectangle(
                            (ox, oy), w, h,
                            edgecolor='black',
                            facecolor='gray',
                            fill=True
                        )
                    )
                for (ox, oy, r) in self.env.obs_circ:
                    ax.add_patch(patches.Circle(
                            (ox, oy), r,
                            edgecolor='black',
                            facecolor='gray',
                            fill=True
                        )
                    )
            plt.title(name)
            plt.axis("equal")
        self.ax.set_title(name)

    def plotExpand(self, expand: list, animate: bool = False, subsample: int = 1) -> None:
        '''
        Optimized plotting for expanded nodes.
        - By default it will plot in one batched call (fast).
        - If animate=True it will append points in chunks to simulate streaming animation but still
          avoid per-point scatter calls.

        subsample: integer factor to sample the expand list (1 == all, 10 == 1/10th)
        '''
        # Defensive copy and filter
        pts = [p for p in expand if p is not None]
        # remove start/goal nodes if present (compare by coordinates)
        pts = [p for p in pts if not (p.x == self.start.x and p.y == self.start.y and (not self.is3d or p.z == getattr(self.start,'z',None)))]
        pts = [p for p in pts if not (p.x == self.goal.x and p.y == self.goal.y and (not self.is3d or p.z == getattr(self.goal,'z',None)))]

        if subsample > 1:
            pts = pts[::subsample]

        if len(pts) == 0:
            return

        # Batch (non-animated) plotting: single scatter call
        if not animate:
            if self.is3d and isinstance(self.env, Grid):
                xs = [p.x for p in pts]
                ys = [p.y for p in pts]
                zs = [p.z for p in pts]
                # plot once
                self._expand_scatter_3d = self.ax.scatter(xs, ys, zs, color="#28a2ff", marker='s', s=squaresize_explored, alpha=0.1)
            elif isinstance(self.env, Grid):
                xs = [p.x for p in pts]
                ys = [p.y for p in pts]
                # Use scatter (returns PathCollection) and store it for potential updates
                self._expand_scatter_2d = self.ax.scatter(xs, ys, color="#cccccc", marker='s', s=squaresize_explored, alpha=0.6)
            # do one draw at the end
            try:
                self.fig.canvas.draw_idle()
            except Exception:
                pass
            return

        # Animated (chunked) plotting: update a single collection in chunks
        chunk_size = 500  # tune this for smoothness vs responsiveness
        if self.is3d and isinstance(self.env, Grid):
            # initialize collection if needed
            if self._expand_scatter_3d is None:
                self._expand_scatter_3d = self.ax.scatter([], [], [], color="#cccccc", marker='s', s=squaresize_explored, alpha=0.6)
                plotted_xs, plotted_ys, plotted_zs = [], [], []
            else:
                # retrieve existing points
                c = self._expand_scatter_3d
                try:
                    plotted_xs, plotted_ys, plotted_zs = list(c._offsets3d[0]), list(c._offsets3d[1]), list(c._offsets3d[2])
                except Exception:
                    plotted_xs, plotted_ys, plotted_zs = [], [], []

            for i in range(0, len(pts), chunk_size):
                chunk = pts[i:i+chunk_size]
                plotted_xs.extend([p.x for p in chunk])
                plotted_ys.extend([p.y for p in chunk])
                plotted_zs.extend([p.z for p in chunk])
                # set offsets for 3d scatter
                try:
                    self._expand_scatter_3d._offsets3d = (plotted_xs, plotted_ys, plotted_zs)
                except Exception:
                    # fallback: re-create scatter (still better than per-point plotting)
                    self._expand_scatter_3d.remove()
                    self._expand_scatter_3d = self.ax.scatter(plotted_xs, plotted_ys, plotted_zs, color="#cccccc", marker='s', s=squaresize_explored, alpha=0.6)
                self.fig.canvas.draw_idle()
                plt.pause(0.001)

        elif isinstance(self.env, Grid):
            # 2D pathcollection update via set_offsets
            if self._expand_scatter_2d is None:
                self._expand_scatter_2d = self.ax.scatter([], [], color="#cccccc", marker='s', s=squaresize, alpha=0.6)
                plotted = np.zeros((0, 2))
            else:
                try:
                    plotted = self._expand_scatter_2d.get_offsets()
                except Exception:
                    plotted = np.zeros((0, 2))

            for i in range(0, len(pts), chunk_size):
                chunk = pts[i:i+chunk_size]
                new_points = np.array([[p.x, p.y] for p in chunk])
                if plotted.size == 0:
                    plotted = new_points
                else:
                    plotted = np.vstack([plotted, new_points])
                self._expand_scatter_2d.set_offsets(plotted)
                self.fig.canvas.draw_idle()
                plt.pause(0.001)

    def plotPath(self, path: list, path_color: str='#13ae00', path_style: str="-") -> None:
        '''
        Plot path in global planning.
        '''
        if self.is3d:
            # Only plot points with valid z values
            path_3d = [p for p in path if len(p) == 3 and p[2] is not None]
            if path_3d:
                path_x = [p[0] for p in path_3d]
                path_y = [p[1] for p in path_3d]
                path_z = [p[2] for p in path_3d]
                self.ax.plot(path_x, path_y, path_z, path_style, linewidth=2, color=path_color)
            # Plot start/goal only if z is not None
            if hasattr(self.start, 'z') and self.start.z is not None:
                self.ax.scatter(self.start.x, self.start.y, self.start.z, marker="s", color="#ff0000")
            if hasattr(self.goal, 'z') and self.goal.z is not None:
                self.ax.scatter(self.goal.x, self.goal.y, self.goal.z, marker="s", color="#1155cc")
            # If no valid 3D points, fallback to 2D
            if not path_3d:
                path_x = [p[0] for p in path]
                path_y = [p[1] for p in path]
                self.ax.plot(path_x, path_y, path_style, linewidth=2, color=path_color)
                self.ax.scatter(self.start.x, self.start.y, marker="s", color="#ff0000")
                self.ax.scatter(self.goal.x, self.goal.y, marker="s", color="#1155cc")
        else:
            path_x = [path[i][0] for i in range(len(path))]
            path_y = [path[i][1] for i in range(len(path))]
            plt.plot(path_x, path_y, path_style, linewidth=2, color=path_color)
            plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000")
            plt.plot(self.goal.x, self.goal.y, marker="s", color="#1155cc")

    def plotAgent(self, pose: tuple, radius: float=1) -> None:
        '''
        Plot agent with specifical pose.

        Parameters
        ----------
        pose: Pose of agent
        radius: Radius of agent
        '''
        x, y, theta = pose
        ref_vec = np.array([[radius / 2], [0]])
        rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta),  np.cos(theta)]])
        end_pt = rot_mat @ ref_vec + np.array([[x], [y]])

        try:
            self.ax.artists.pop()
            for art in self.ax.get_children():
                if isinstance(art, matplotlib.patches.FancyArrow):
                    art.remove()
        except:
            pass

        self.ax.arrow(x, y, float(end_pt[0]) - x, float(end_pt[1]) - y,
                width=0.1, head_width=0.40, color="r")
        circle = plt.Circle((x, y), radius, color="r", fill=False)
        self.ax.add_artist(circle)

    def plotHistoryPose(self, history_pose, predict_path=None, lookahead_pts=None) -> None:
        lookahead_handler = None
        for i, pose in enumerate(history_pose):
            if i < len(history_pose) - 1:
                plt.plot([history_pose[i][0], history_pose[i + 1][0]],
                    [history_pose[i][1], history_pose[i + 1][1]], c="#13ae00")
                if predict_path is not None:
                    plt.plot(predict_path[i][:, 0], predict_path[i][:, 1], c="#ddd")
            i += 1

            # agent
            self.plotAgent(pose)

            # lookahead
            if lookahead_handler is not None:
                lookahead_handler.remove()
            if lookahead_pts is not None:
                try:
                    lookahead_handler = self.ax.scatter(lookahead_pts[i][0], lookahead_pts[i][1], c="b")
                except:
                    lookahead_handler = self.ax.scatter(lookahead_pts[-1][0], lookahead_pts[-1][1], c="b")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event: [exit(0) if event.key == 'escape' else None])
            if i % 5 == 0:
                plt.pause(0.03)

    def plotCostCurve(self, cost_list: list, name: str) -> None:
        '''
        Plot cost curve with epochs using in evolutionary searching.

        Parameters
        ----------
        cost_list: Cost with epochs
        name: Algorithm name or some other information
        '''
        plt.plot(cost_list, color="b")
        plt.xlabel("epochs")
        plt.ylabel("cost value")
        plt.title(name)
        plt.grid()

    def plotEllipse(self, ellipse: np.ndarray, color: str = 'darkorange', linestyle: str = '--', linewidth: float = 2):
        plt.plot(ellipse[0, :], ellipse[1, :], linestyle=linestyle, color=color, linewidth=linewidth)

    def connect(self, name: str, func) -> None:
        self.fig.canvas.mpl_connect(name, func)

    def clean(self):
        plt.cla()

    def update(self):
        self.fig.canvas.draw_idle()

    @staticmethod
    def plotArrow(x, y, theta, length, color):
        angle = np.deg2rad(30)
        d = 0.5 * length
        w = 2

        x_start, y_start = x, y
        x_end = x + length * np.cos(theta)
        y_end = y + length * np.sin(theta)

        theta_hat_L = theta + np.pi - angle
        theta_hat_R = theta + np.pi + angle

        x_hat_start = x_end
        x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

        y_hat_start = y_end
        y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

        plt.plot([x_start, x_end], [y_start, y_end], color=color, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_L], [y_hat_start, y_hat_end_L], color=color, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_R], [y_hat_start, y_hat_end_R], color=color, linewidth=w)

    @staticmethod
    def plotCar(x, y, theta, width, length, color):
        theta_B = np.pi + theta

        xB = x + length / 4 * np.cos(theta_B)
        yB = y + length / 4 * np.sin(theta_B)

        theta_BL = theta_B + np.pi / 2
        theta_BR = theta_B - np.pi / 2

        x_BL = xB + width / 2 * np.cos(theta_BL)        # Bottom-Left vertex
        y_BL = yB + width / 2 * np.sin(theta_BL)
        x_BR = xB + width / 2 * np.cos(theta_BR)        # Bottom-Right vertex
        y_BR = yB + width / 2 * np.sin(theta_BR)

        x_FL = x_BL + length * np.cos(theta)               # Front-Left vertex
        y_FL = y_BL + length * np.sin(theta)
        x_FR = x_BR + length * np.cos(theta)               # Front-Right vertex
        y_FR = y_BR + length * np.sin(theta)

        plt.plot([x_BL, x_BR, x_FR, x_FL, x_BL],
                 [y_BL, y_BR, y_FR, y_FL, y_BL],
                 linewidth=1, color=color)

        Plot.plotArrow(x, y, theta, length / 2, color)
