#!/usr/bin/env python
# coding: utf-8


from vedo import *

def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
	
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij
	
import numpy as np

def forward_kinematics(Phi, L1, L2, L3, L4):
    original_location = np.array([[3], [2],[0]])
    radius = 0.4
    R_01 = RotationMatrix(Phi[0], 'z')
    T_01 = original_location
    T_01 = getLocalFrameMatrix(R_01,T_01)
    
   
    T_12 = np.array([[L1 + (2*radius)],[0.0],[0.0]])
    R_12 = RotationMatrix(Phi[1], 'z')
    T_12 = getLocalFrameMatrix(R_12,T_12)
    T_02 =  T_01 @ T_12
    
    # Second joint to third joint
    T_23 =np.array([[L2 + (2*radius)],[0.0],[0.0]])
    R_23 = RotationMatrix(Phi[2], 'z')
    T_23 = getLocalFrameMatrix(R_23,T_23)
    T_03 = T_02 @ T_23
    
    # Third joint to end-effector
    T_34 = np.array([[L3 + radius],[0.0],[0.0]])
    R_34 = RotationMatrix(Phi[3], 'z')
    T_34 = getLocalFrameMatrix(R_34,T_34)
    T_04 = T_03 @ T_34
    
    # End-effector location
    e = T_04[:3, 3]
    
    return T_01, T_02, T_03, T_04, e



def main():
    R = 0.4
    # Set the limits of the graph x, y, and z ranges 
    axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))

    # Lengths of arm parts 
    L1 = 5   # Length of link 1
    L2 = 8   # Length of link 2
    L3 = 3   # Length of link 3
    L4 = 0   # Length of the end-effector

    # Joint_angles 
    phi1 = 40     # Rotation angle of part 1 in degrees
    phi2 = -80      # Rotation angle of part 2 in degrees
    phi3 = 80      # Rotation angle of part 3 in degrees
    phi4 = -120     # Rotation angle of the end-effector in degrees

    Phi = np.array([phi1, phi2, phi3, phi4])
    T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)

    # Create the coordinate frame mesh for Frame 1 and transform
    Frame1Arrows = createCoordinateFrameMesh()
    Frame1Arrows.apply_transform(T_01)
    
    # Create the link mesh for Frame 1
    link1_mesh = Cylinder(r=0.4, height=L1,
                           pos=((L1/2)+ R,0 ,0 ),
                             c="blue", 
                             alpha=.8, 
                             axis=(1,0,0))
    link1_mesh.apply_transform(T_01)

    # Create the joint sphere for Frame 1
    joint1_sphere = Sphere(r=0.4).pos(T_01[:3, 3]).color("red").alpha(.8)

    # Create the complete Frame 1 with arrows and link mesh
    Frame1 =  link1_mesh + joint1_sphere + Frame1Arrows

    # Repeat the process for Frame 2
    Frame2Arrows = createCoordinateFrameMesh()
    Frame2Arrows.apply_transform(T_02)
    link2_mesh = Cylinder(r=0.4, height=L2, pos=((L2/2) + R ,0,0), c="blue", alpha=.8, axis=(1,0,0))
    link2_mesh.apply_transform(T_02)
    joint2_sphere = Sphere(r=0.4).pos(T_02[:3, 3]).color("red").alpha(.8)
    Frame2 =  link2_mesh + joint2_sphere + Frame2Arrows

    # Repeat the process for Frame 3
    Frame3Arrows = createCoordinateFrameMesh()
    Frame3Arrows.apply_transform(T_03)
    link3_mesh = Cylinder(r=0.4, height=L3, pos=((L3/2)+ R,0,0), c="blue", alpha=.8, axis=(1,0,0))
    link3_mesh.apply_transform(T_03)
    joint3_sphere = Sphere(r=0.4).pos(T_03[:3, 3]).color("red").alpha(.8)
    Frame3 = link3_mesh + joint3_sphere + Frame3Arrows 

    # For the end-effector (Frame 4), we only need the coordinate frame arrows
    Frame4Arrows = createCoordinateFrameMesh()
    Frame4Arrows.apply_transform(T_04)
    Frame4 = Frame4Arrows  
 
    show([Frame1, Frame2, Frame3, Frame4, axes], axes, viewup="z").close()

if __name__ == '__main__':
    main()