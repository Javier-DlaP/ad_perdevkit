import numpy as np
import math

def ros_intrinsics(intrinsics):

    intrins_matrix = np.zeros((3,4))
    intrins_matrix[0,:] = intrinsics[0:4] 
    intrins_matrix[1,:] = intrinsics[4:8] 
    intrins_matrix[2,:] = intrinsics[8:12] 

    return intrins_matrix

def compute_box_3d(pos, dim, rot):

    # compute rotational matrix around yaw axis
    R = self.roty(rot)    

    # 3d bounding box dimensions
    l, w, h = dim
    
    # 3d bounding box corners
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h]
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    
    # rotate and translate 3d bounding box
    corners_3d = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    #print corners_3d.shape
    corners_3d[0,:] = corners_3d[0,:] + pos[0]
    corners_3d[1,:] = corners_3d[1,:] + pos[1]
    corners_3d[2,:] = corners_3d[2,:] + pos[2]
    #print 'cornsers_3d: ', corners_3d 
    # only draw 3d bounding box for objs in front of the camera
    if np.any(corners_3d[2,:]<0.1):
        corners_2d = None
        return corners_2d, np.transpose(corners_3d)
    
    #print 'corners_2d: ', corners_2d
    return np.transpose(corners_3d)

def rotx(t):
    
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1,  0,  0],
                     [0,  c, -s],
                     [0,  s,  c]])

def roty(t):
    
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0,  s],
                     [0,  1,  0],
                     [-s, 0,  c]])

def rotz(t):
    
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  -s,  0],
                     [s,   c,  0],
                     [0,   0,  1]])

def normalizeAngle(angle):

    while angle < -np.pi:
        angle += 2 * np.pi
    while angle > np.pi:
        angle -= 2 * np.pi
    return angle

def pointInBB3D_rotation(point3D, bb3D):
    
    x, y, z = point3D
    x_bb, y_bb, z_bb = bb3D[0:3]
    l, w, h = bb3D[3:6]
    rotation_z = bb3D[6]

    x_rel = x - x_bb
    y_rel = y - y_bb

    # x_rotated = x_rel * math.sin(rotation_z) - y_rel * math.cos(rotation_z)
    # y_rotated = - (x_rel * math.sin(rotation_z) + y_rel * math.cos(rotation_z))

    x_rotated = x_rel * math.cos(rotation_z) - y_rel * math.sin(rotation_z)
    y_rotated = x_rel * math.sin(rotation_z) + y_rel * math.cos(rotation_z)

    if abs(x_rotated) > w/2 or abs(y_rotated) > l/2:
        return False
    else:
        return True

def bb3DVertices(bb3D):

    x, y, z = bb3D[0:3]
    h, w, l = bb3D[3:6]
    rotation_z = bb3D[6]

    vertices = np.transpose(np.matrix([[-w/2, h/2, -l/2, 1], [-w/2, h/2, l/2, 1], [-w/2, -h/2, l/2, 1], [-w/2, -h/2, -l/2, 1],
                                       [w/2, h/2, -l/2, 1], [w/2, h/2, l/2, 1], [w/2, -h/2, l/2, 1], [w/2, -h/2, -l/2, 1]]))

    c = np.cos(rotation_z-np.pi/2)
    s = np.sin(rotation_z-np.pi/2)
    rotation_matrix = np.array([[ c, 0, s, 0],
                                [ 0, 1, 0, 0],
                                [-s, 0, c, 0],
                                [ 0, 0, 0, 1]])

    rotated_vertices = np.dot(rotation_matrix, vertices)

    final_vertices = np.transpose(rotated_vertices) + np.array([x, y, z, 0])

    return final_vertices