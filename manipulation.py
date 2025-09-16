from ikpy.chain import Chain
from ikpy.link import DHLink
import numpy as np
import sympy as sp
from sympy import Matrix as M, sin, cos
from mpmath import radians
from scipy.integrate import ode
import matplotlib.pyplot as plt

class Manipulator:

    def __init__(self, ):
        self.thetas = sp.symbols("t1 t2")
        self.l1 = 1.0
        self.l2 = 1.0
        self.dh_params = np.array([[""" Your DH Params Go Here """]]) # 2D array of DH Parameters
        
        self.planar_chain = Chain(name='2D_Planar_Arm', links=[  
                                                    DHLink(
                                                            name="link"+str(i+1),
                                                            a= dhp[2],
                                                            d=dhp[3]
                                                        )                                                       
                                                        for i, dhp in enumerate(self.dh_params)
                                                ]
        )

        # End Effector Equations as Sympy Matrix 
        self.end_eff_eqs = self.get_end_effector_equations(self.dh_params)

        # Jacobian Equations as Sympy Matrix
        self.J = self.get_Jacobian_from_end_eqs(self.end_eff_eqs, self.thetas)

        self.cur_pos = None
        self.target_pos = None
        self.waypoints = None

    # Given a DH Matrix, gives the Total transformation matrix from baselink to the end effector
    def get_transform_from_dh_matrix(self, dh_params: sp.Matrix):
        final_tf_matrix = sp.eye(4)
        len = dh_params.shape[0]
        for i in range(len):
            dh_i = dh_params[i,:]
            t, al, a, d = dh_i
            t = t
            al = al
            tf_mat = M([
                        [cos(t), -sin(t)*cos(al), sin(t)*sin(al), a*cos(t)],
                        [sin(t), cos(t)*cos(al), -cos(t)*sin(al), a*sin(t)],
                        [0,      sin(al),         cos(al),          d],
                        [0,           0,                 0,         1]
                    ])

            final_tf_matrix = final_tf_matrix * tf_mat
        return final_tf_matrix
    
    # This method returns the end effecttor coordinates in the form of M([[xe], [ye], [ze]]), where each coordinate is 
    # an equation in terms of t1 and t2, e.g., xe = l1*cos(t1) + l2*cos(t1 + t2)
    def get_end_effector_equations(self, dh_params):
        final_tf = self.get_transform_from_dh_matrix(dh_params)
        return final_tf[:3,-1]


    def get_Jacobian_from_end_eqs(self, end_eff_eqs: sp.MutableDenseMatrix, angles):#the symbolic end effector equation and the symbolic angles
        J = []  # The Jacobian should be a 2D list of equations
        ######################################################
        #
        # Your code goes here
        # Use Sympy calculus to derive the Jacobian from the end effector equations and the symbolic angle variables (thetas = [t1, t2])
        #
        ######################################################
        return M(J)
    
    # Returns the end effector position, given the joint angles
    def get_end_effector_pos_from_angles(self, angles):
        return np.squeeze(np.array(self.end_eff_eqs.subs({'t1':angles[0], 't2':angles[1]}).evalf(), dtype=float))
    
    # This function returns the joint velocities as a ndarray
    def pid_control(self, t, cur_angles):  #Can be P, PD, or PID control
        q_dot = None
        ###############################################################
        # Your code for calculating the control goes here
        #
        # To evaluate the Jacobian for the particular angles, you can use
        # J_q = np.array(self.J.subs({'t1':cur_angles[0], 't2':cur_angles[1]}).evalf(), dtype = float)
        # Follow the Inverse Kinematics section at https://realitybytes.blog/2017/06/20/forward-and-inverse-kinematics-jacobians-and-differential-motion/
        # IMPORTANT: t is not utilized for this section. The integrator requires it
        ###############################################################
        return q_dot
    
    def start(self, init_pos, target_pos):
        cur_angles = self.planar_chain.inverse_kinematics(init_pos)
        self.cur_pos = init_pos
        self.target_pos = target_pos

        integrator = ode(self.pid_control).set_integrator('dopri5')
        integrator.set_initial_value(cur_angles, 0)
        dt = 0.001
        self.waypoints = [cur_angles]
        while integrator.successful() : # Add Termination criteria
            q = integrator.integrate(integrator.t + dt)
            self.waypoints.append(q)
            t = integrator.t
            # Reference code for getting the current position
            self.cur_pos = self.get_end_effector_pos_from_angles(q)
        self.plot_waypoints()
        
    def plot_waypoints(self, ):
        fig, ax = plt.subplots()
        self.waypoints = np.array(self.waypoints)
        eeffpath = np.array([ self.get_end_effector_pos_from_angles(q) for q in self.waypoints])
        nominal_traj, = ax.plot(eeffpath[:,0],eeffpath[:,1], '-', linewidth='2', color='orange', label="Trajectory")
        manip_line, = ax.plot([], [], 'bo-', alpha=0.4, linewidth=4, label='Manipulator Joint')
        endeff_wp, = ax.plot([], [],'go', label='End Effector Waypoint')

        len_arr = len(self.waypoints)
        pts = [0, len_arr//10, len_arr//5, len_arr-1]
        for i in pts:
            angles = self.waypoints[i]
            print(f"Angles {i} : {angles}")
            joint1_x, joint1_y = self.l1 * np.cos(angles[0]), self.l1 * np.sin(angles[0])
            end_effector_x, end_effector_y = eeffpath[i,0], eeffpath[i,1]
            ax.plot([0, joint1_x, end_effector_x], [0, joint1_y, end_effector_y], 'bo-', alpha=0.4, linewidth=4)   
            ax.plot([end_effector_x], [end_effector_y],'go')

        ax.set_xticks(np.arange(-0.1, 2.1, 0.25))
        ax.set_yticks(np.arange(0, 2.1, 0.25))
            
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.grid()
        plt.show()


def main(args=None):
    manipulator = Manipulator()
    init_pos = np.array([0, 2, 0])
    target_pos = np.array([2, 0, 0])
    manipulator.start(init_pos=init_pos, target_pos=target_pos)

if __name__ == "__main__":
    main()
