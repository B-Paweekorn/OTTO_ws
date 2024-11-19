import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
class FourBarMechanism:
    def __init__(self, L1, L2, L3, L4, L5, MODE, t1_init, t2_init):
        self.mode = MODE

        self.L1 = L1  # (Input) A-D link
        self.L2 = L2  # (Input) A-B link
        self.L3 = L3  # B-C link
        self.L4 = L4  # C-D link
        self.L5 = L5  # D-E link (extension)
        
        self.t1 = t1_init
        self.t2 = t2_init
        self.t1_init = t1_init  # Initial angle for self.t1
        self.t2_init = t2_init  # Initial angle for self.t2

        self.kp = 1

        # Set up the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)
        self.ax.grid(True)  # Add grid to the plot

        # Links and end-effector
        self.link1, = self.ax.plot([], [], 'k-', lw=2)
        self.link2, = self.ax.plot([], [], 'b-', lw=2)
        self.link3, = self.ax.plot([], [], 'k-', lw=2)
        self.link4, = self.ax.plot([], [], 'b-', lw=2)
        self.link5, = self.ax.plot([], [], 'g-', lw=2)
        self.star, = self.ax.plot([], [], marker='*', color='blue', markersize=3)  # End-effector

        # Create sliders
        ax_slider_x = plt.axes([0.20, 0.95, 0.65, 0.02], facecolor='lightgoldenrodyellow')
        self.slider_x = Slider(ax_slider_x, 'targ_x', -10.0, 10.0, valinit=0)

        ax_slider_y = plt.axes([0.20, 0.91, 0.65, 0.02], facecolor='lightgoldenrodyellow')
        self.slider_y = Slider(ax_slider_y, 'targ_y', -10.0, 10.0, valinit=-2.2)

        self.slider_x.on_changed(self.update_target)
        self.slider_y.on_changed(self.update_target)

        self.target_x = self.slider_x.val
        self.target_y = self.slider_y.val
    
        # Connect the click event
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

    def on_click(self, event):
        """Handle mouse click events to set target coordinates."""
        if event.inaxes == self.ax and self.mode == "jac":
            self.target_x = event.xdata
            self.target_y = event.ydata
            print(f"Target updated: x={self.target_x}, y={self.target_y}")

    def update_target(self, val):
        if self.mode == "inv":
            self.target_x = self.slider_x.val
            self.target_y = self.slider_y.val

    def solve_fourbar(self):
        A = np.array([0, 0])
        B = np.array([self.L2 * np.cos(self.t2), self.L2 * np.sin(self.t2)])
        C = np.array([B[0] + self.L3 * np.cos(self.t1), B[1] + self.L3 * np.sin(self.t1)])
        D = np.array([self.L1 * np.cos(self.t1), self.L1 * np.sin(self.t1)])
        E = np.array([D[0] - self.L5 * np.cos(self.t2), D[1] - self.L5 * np.sin(self.t2)])

        return A, B, C, D, E

    def forward_kin(self):
        return np.array([self.L1 * np.cos(self.t1) - self.L5 * np.cos(self.t2),
                         self.L1 * np.sin(self.t1) - self.L5 * np.sin(self.t2)])
    
    def fnc_jacobian(self):
        J = np.array([[-L1*np.sin(self.t1), L5*np.sin(self.t2)], [L1*np.cos(self.t1), -L5*np.cos(self.t2)]])
        J_inv = np.linalg.inv(J)
        return J, J_inv

    def inverse_kin(self, x, y):
        t1 = np.arctan2(-y, x) - np.angle((abs(L1) * abs(L5) * (x**2 - L5**2 - L1**2 + y**2 + 2 * L1 * L5 * 
            np.imag(np.sqrt(-(L1**4 - 2 * L1**2 * L5**2 - 2 * L1**2 * x**2 - 2 * L1**2 * y**2 + L5**4 - 
            2 * L5**2 * x**2 - 2 * L5**2 * y**2 + x**4 + 2 * x**2 * y**2 + y**4) / (4 * L1**2 * L5**2)))+ 
            L1 * L5 * np.real(np.sqrt(-(L1**4 - 2 * L1**2 * L5**2 - 2 * L1**2 * x**2 - 2 * L1**2 * y**2 + 
            L5**4 - 2 * L5**2 * x**2 - 2 * L5**2 * y**2 + x**4 + 2 * x**2 * y**2 + y**4) / (4 * L1**2 * L5**2))) * 2j + 
            (L1**2 * abs(L1**2 + L5**2 - x**2 - y**2 + L1 * L5 * np.sqrt(-(L1**4 - 2 * L1**2 * L5**2 - 2 * L1**2 * x**2 - 2 * L1**2 * y**2 + L5**4 - 
            2 * L5**2 * x**2 - 2 * L5**2 * y**2 + x**4 + 2 * x**2 * y**2 + y**4) / (4 * L1**2 * L5**2)) * 2j)) / 
            (abs(L1) * abs(L5)))) / (L1 * abs(L1**2 + L5**2 - x**2 - y**2 + L1 * L5 * 
            np.sqrt(-(L1**4 - 2 * L1**2 * L5**2 - 2 * L1**2 * x**2 - 2 * L1**2 * y**2 + L5**4 - 
            2 * L5**2 * x**2 - 2 * L5**2 * y**2 + x**4 + 2 * x**2 * y**2 + y**4) / (4 * L1**2 * L5**2)) * 2j)))

        t2 = np.angle((L1**2 + L5**2 - x**2 - y**2 + L1*L5*(-(L1**4 - 2*L1**2*L5**2 - 2*L1**2*x**2 - 2*L1**2*y**2 + L5**4 - 2*L5**2*x**2 - 2*L5**2*y**2 + x**4 + 2*x**2*y**2 + y**4) / 
             (4*L1**2*L5**2))**(1/2) * 2j) / (L1 * L5)) - np.arctan2(-y, x) + np.angle((np.abs(L1) * np.abs(L5) * (x**2 - L5**2 - L1**2 + y**2 + 2*L1*L5*np.imag((-(L1**4 - 2*L1**2*L5**2 - 2*L1**2*x**2 - 2*L1**2*y**2 + L5**4 - 2*L5**2*x**2 - 2*L5**2*y**2 + x**4 + 2*x**2*y**2 + y**4) / \
             (4*L1**2*L5**2))**(1/2)) + L1*L5*np.real((-(L1**4 - 2*L1**2*L5**2 - 2*L1**2*x**2 - 2*L1**2*y**2 + L5**4 - 2*L5**2*x**2 - 2*L5**2*y**2 + x**4 + 2*x**2*y**2 + y**4) / (4*L1**2*L5**2))**(1/2)) * 2j + 
             (L1**2 * np.abs(L1**2 + L5**2 - x**2 - y**2 + L1*L5*(-(L1**4 - 2*L1**2*L5**2 - 2*L1**2*x**2 - 2*L1**2*y**2 + L5**4 - 2*L5**2*x**2 - 2*L5**2*y**2 + x**4 + 2*x**2*y**2 + y**4) / (4*L1**2*L5**2))**(1/2) * 2j)) / (np.abs(L1) * np.abs(L5)))) / \
             (L1 * np.abs(L1**2 + L5**2 - x**2 - y**2 + L1*L5*(-(-(L1**4 - 2*L1**2*L5**2 - 2*L1**2*x**2 - 2*L1**2*y**2 + L5**4 - 2*L5**2*x**2 - 2*L5**2*y**2 + x**4 + 2*x**2*y**2 + y**4) / (4*L1**2*L5**2))**(1/2) * 2j))))
        print(np.rad2deg(t1), np.rad2deg(t2))
        return -t1, t2
    
    def controller(self, curr ,targ):
        if abs(np.linalg.det(self.fnc_jacobian()[0])) < 0.1:
            print("Singularlity!!")
            self.mode = "inv"
            return np.array([0, 0])
        err = targ - curr # 1 x 2
        v = err * self.kp
        q_dot = np.matmul(self.fnc_jacobian()[1], np.transpose(v))
        # print(q_dot)
        return q_dot

    def init(self):
        self.link1.set_data([], [])
        self.link2.set_data([], [])
        self.link3.set_data([], [])
        self.link4.set_data([], [])
        self.link5.set_data([], [])
        self.star.set_data([], [])
        return self.link1, self.link2, self.link3, self.link4, self.link5, self.star

    def animate(self, i):
        if self.mode == "inv":
            targ_x = self.slider_x.val
            targ_y = self.slider_y.val
            self.t1, self.t2 = self.inverse_kin(targ_x, targ_y)
        if self.mode == "jac":
            p_c = self.forward_kin()
            p_t = np.array([self.target_x, self.target_y]) 
            q_dot = self.controller(p_c, p_t)
            self.t1 = self.t1 + q_dot[0] * 0.1
            self.t2 = self.t2 + q_dot[1] * 0.1

        A, B, C, D, E = self.solve_fourbar()

        self.link1.set_data([A[0], D[0]], [A[1], D[1]])
        self.link2.set_data([A[0], B[0]], [A[1], B[1]])
        self.link3.set_data([B[0], C[0]], [B[1], C[1]])
        self.link4.set_data([C[0], D[0]], [C[1], D[1]])
        self.link5.set_data([D[0], E[0]], [D[1], E[1]])

        eff = self.forward_kin()
        self.star.set_data(eff[0], eff[1])

        return self.link1, self.link2, self.link3, self.link4, self.link5, self.star

    def run(self):
        # Create animation
        ani = FuncAnimation(self.fig, self.animate, frames=10000, init_func=self.init, blit=False, interval=0.1)
        plt.show()


L1 = 2.0  # (Input) A-D link
L2 = 0.8  # (Input) A-B link
L3 = 2.0  # B-C link
L4 = 0.8  # C-D link
L5 = 2.0  # D-E link (extension)
t1_init = np.deg2rad(-45)
t2_init = np.deg2rad(45)

MODE = "jac"

mechanism = FourBarMechanism(L1, L2, L3, L4, L5, MODE, t1_init, t2_init)
mechanism.run()