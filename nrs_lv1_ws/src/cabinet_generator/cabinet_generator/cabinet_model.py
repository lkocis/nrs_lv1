
from xml.dom import minidom
import xml.etree.ElementTree as gfg
import os
from math import pi
import numpy as np
from tf_transformations import euler_from_matrix
from copy import deepcopy

OLD = True

def rot_z(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    return R

def rot_y(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, 0, s],
                  [0, 1, 0],
                  [-s, 0, c]])
    return R


class Cabinet():
    def __init__(self, door_params:np.array=np.array([0.1, 0.1, 0.018, 0.1]), 
                 r:np.array=np.array([0, 0]),
                 axis_pos: int=1, 
                 T_A_S: np.ndarray=np.eye(4), 
                 save_path:str=None,
                 has_handle:bool=False,
                 initial_angle_deg:float=0.
                 ):

        self.w_door = door_params[0]
        self.h_door = door_params[1]
        self.d_door = door_params[2]
        self.static_d = door_params[3]
        self.moving_to_static_part_distance = 0.005
        self.axis_distance = 0.0
        self.static_side_width = 0.018
        self.initial_angle_deg = initial_angle_deg
        self.initial_angle_rad = np.radians(initial_angle_deg)

        self.rx = r[0]
        self.ry = r[1]

        self.has_handle = has_handle
        self.handle_radius = 0.005
        self.handle_length = 0.08
        self.handle_height = 0.02
        self.handle_base_radius = 0.003
        self.handle_offset = 0.05

        self.base_link_name = 'base_cabinet_link'
        self.door_panel_link_name = 'door_link'
        self.door_joint_name = 'door_joint'
        self.cabinet_name = 'my_cabinet'
        self.handle_link_name  = 'handle_link'
        self.handle_joint_name  = 'handle_joint'
        self.save_path = save_path

        self.T_A_S = T_A_S
        self.current_angle_state_deg = 0.
        if not axis_pos == 1 and not axis_pos == -1:
            self.axis_pos = 1
        else:
            self.axis_pos = axis_pos

        self.setup_matrices()

        self.xml_model = None
        self.root_urdf = None
        if self.save_path is not None:
            self.xml_model = self.generate_cabinet_urdf_from_door_panel()

    def setup_matrices(self):
        self.T_F_O = np.eye(4)
        self.T_F_O[:3, :3] = np.array([[0, 0, -1],
                                [1, 0, 0],
                                [0, -1, 0]])
        self.T_F_O[:3, 3] = np.array([self.static_d/2.,
                               -(self.w_door/2. + 2*self.moving_to_static_part_distance + self.d_door),
                                self.h_door/2. + self.moving_to_static_part_distance + self.d_door])


        self.T_A_O = np.eye(4)
        if self.axis_pos < 0:
            Tz = np.eye(4)
            Tz[:3, :3] = rot_z(np.pi)
            self.T_A_O = Tz @ self.T_A_O

        if OLD:
            self.T_A_O[:3, 3] = np.array([self.static_d/2. - self.d_door/2., 
                                          self.axis_pos*(self.w_door/2. - self.axis_distance),  # Old - working
                                        0.])
        else:
            self.T_A_O[:3, 3] = np.array([self.static_d/2. - self.d_door/2., 
                                        self.axis_pos*(self.w_door/2. - 0), # New - in coll when fully open
                                        0.])

        self.T_A_O_init = self.T_A_O.copy()


        self.T_O_S = self.T_A_S @ np.linalg.inv(self.T_A_O_init)

        self.T_D_A = np.eye(4)
        self.T_D_A[:3, :3] = np.array([[0, 0, -self.axis_pos],
                                      [self.axis_pos, 0, 0],
                                      [0, -1, 0]])
        self.T_D_A[:3, 3] = np.array([-self.axis_pos*self.d_door*0.5,
                                      -self.w_door,
                                      self.h_door*0.5])
        
        self.T_D_A_init = self.T_D_A.copy()
        self.T_D_A_init[:3, 3] = np.array([-self.axis_pos*self.d_door*0.5,
                                      -self.w_door,
                                      self.h_door*0.5])
        



        self.T_H_A = np.eye(4) # Handle in axis frame
        self.T_H_A[:3,  3] = np.array([self.axis_pos*(self.handle_height+self.d_door/2.), 
                                  -1*(self.w_door -self.handle_offset), 
                                  0.0])


    def generate_cabinet_urdf_from_door_panel(self):
        static_d = self.static_d
        moving_to_static_part_distance = self.moving_to_static_part_distance
        axis_distance = self.axis_distance
        static_side_width = self.static_side_width
        bottom_panel_height = static_side_width
        w = self.w_door + 2*(moving_to_static_part_distance + static_side_width) + moving_to_static_part_distance
        h = self.h_door + 2*moving_to_static_part_distance + static_side_width + bottom_panel_height
        d = self.d_door
        base_link_name = self.base_link_name
        door_panel_link_name = self.door_panel_link_name
        axis_pos = self.axis_pos

        save_path = self.save_path

        panel_dims = np.array([[static_d, self.w_door+3*moving_to_static_part_distance+2*static_side_width, bottom_panel_height], # panel bottom
                            [static_d, d, h], # side
                            [static_d, d, h], # side
                            [static_d, w, d], # top
                            [d, w, h]]) # back

        panel_positions = np.array([[0., -moving_to_static_part_distance*0.5, -self.h_door*0.5 - moving_to_static_part_distance - bottom_panel_height*0.5], # bottom panel
                        [0., self.w_door*0.5 + moving_to_static_part_distance + static_side_width*0.5, 0], # side panel
                        [0., -self.w_door*0.5 - 2*moving_to_static_part_distance - static_side_width*0.5, 0], # side panel
                        [0., -moving_to_static_part_distance*0.5, self.h_door*0.5+moving_to_static_part_distance+d*0.5], # top
                        [-(static_d/2.-d/2.), -moving_to_static_part_distance*0.5, 0]]) # back panel


        self.root_urdf = gfg.Element('robot')
        self.root_urdf.set('name', self.cabinet_name)

        world_link = gfg.SubElement(self.root_urdf, 'link')
        world_link.set('name', 'world')

        virtual_joint = gfg.SubElement(self.root_urdf, 'joint')
        virtual_joint.set('name', 'virtual_joint')
        virtual_joint.set('type', 'fixed')
        vj_parent = gfg.SubElement(virtual_joint, 'parent')
        vj_parent.set('link', 'world')
        vj_child = gfg.SubElement(virtual_joint, 'child')
        vj_child.set('link', base_link_name)
        vj_origin = gfg.SubElement(virtual_joint, 'origin')
        vj_origin.set('xyz', '0.0 0.0 0.0')
        vj_origin.set('rpy', '0.0 0.0 0.0')

        base_door_link = gfg.SubElement(self.root_urdf, 'link')
        base_door_link.set('name', base_link_name)

        for i in range(panel_positions.shape[0]):
            panel_visual = gfg.SubElement(base_door_link, 'visual')

            pl_visual_origin = gfg.SubElement(panel_visual, 'origin')
            pl_visual_origin.set('xyz', '{} {} {}'.format(panel_positions[i, 0], panel_positions[i, 1], panel_positions[i, 2]))
            pl_visual_origin.set('rpy', '0.0 0.0 0.0')

            pl_visual_geom = gfg.SubElement(panel_visual, 'geometry')
            pl_visual_geom_box = gfg.SubElement(pl_visual_geom, 'box')
            pl_visual_geom_box.set('size', '{} {} {}'.format(panel_dims[i, 0], panel_dims[i, 1], panel_dims[i, 2]))

            panel_collision = gfg.SubElement(base_door_link, 'collision')
            panel_collision.set('name', base_link_name+'_collision_%d' % i)
            pl_collision_origin = gfg.SubElement(panel_collision, 'origin')
            pl_collision_origin.set('xyz', '{} {} {}'.format(panel_positions[i, 0], panel_positions[i, 1], panel_positions[i, 2]))
            pl_collision_origin.set('rpy', '0.0 0.0 0.0')

            pl_collision_geom = gfg.SubElement(panel_collision, 'geometry')
            pl_collision_geom_box = gfg.SubElement(pl_collision_geom, 'box')
            pl_collision_geom_box.set('size', '{} {} {}'.format(panel_dims[i, 0], panel_dims[i, 1], panel_dims[i, 2]))

        pl_inertial = gfg.SubElement(base_door_link, 'inertial')
        m = 0.2
        ixx = 1/12.*m*(w**2 + h**2)
        iyy = 1/12.*m*(static_d**2 + h**2)
        izz = 1/12.*m*(static_d**2 + w**2)
        pl_inertial_mass = gfg.SubElement(pl_inertial, 'mass')
        pl_inertial_mass.set('value', '%f' % m)
        pl_inertial_inertia = gfg.SubElement(pl_inertial, 'inertia')
        pl_inertial_inertia.set('ixx', '%f' % ixx)
        pl_inertial_inertia.set('ixy', '0.0')
        pl_inertial_inertia.set('ixz', '0.0')
        pl_inertial_inertia.set('iyy', '%f' % iyy)
        pl_inertial_inertia.set('iyz', '0.0')
        pl_inertial_inertia.set('izz', '%f' % izz)


        door_panel_dims = [self.d_door, self.w_door, self.h_door]
        door_panel_position = [static_d/2. - d/2., 0., 0.]

        door_panel_link = gfg.SubElement(self.root_urdf, 'link')
        door_panel_link.set('name', 'door_link')

        door_panel_visual = gfg.SubElement(door_panel_link, 'visual')
        dpl_visual_origin = gfg.SubElement(door_panel_visual, 'origin')
        
        if OLD:
            dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(-1*(door_panel_dims[1]/2. - axis_distance)))
        else:
            if self.axis_pos == 1:
                dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))
            else:
                dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))

        dpl_visual_geom = gfg.SubElement(door_panel_visual, 'geometry')
        dpl_visual_geom_box = gfg.SubElement(dpl_visual_geom, 'box')
        dpl_visual_geom_box.set('size', '{} {} {}'.format(door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))

        door_panel_collision = gfg.SubElement(door_panel_link, 'collision')
        door_panel_collision.set('name', self.door_panel_link_name+'_collision')
        dpl_collision_origin = gfg.SubElement(door_panel_collision, 'origin')
        
        if OLD:
            dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(-1*(door_panel_dims[1]/2. - axis_distance)))
        else:
            if self.axis_pos == 1:
                dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))
            else:
                dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))
        
        dpl_collision_geom = gfg.SubElement(door_panel_collision, 'geometry')
        dpl_collision_geom_box = gfg.SubElement(dpl_collision_geom, 'box')
        dpl_collision_geom_box.set('size', '{} {} {}'.format(door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))


        dpl_inertial = gfg.SubElement(door_panel_link, 'inertial')
        # The panel inertia is about its centre, not the hinge.
        gfg.SubElement(dpl_inertial, 'origin', dict(dpl_collision_origin.attrib))
        m = 1.0
        ixx = 1/12.*m*(door_panel_dims[1]**2 + door_panel_dims[2]**2)
        iyy = 1/12.*m*(door_panel_dims[0]**2 + door_panel_dims[2]**2)
        izz = 1/12.*m*(door_panel_dims[0]**2 + door_panel_dims[1]**2)
        dpl_inertial_mass = gfg.SubElement(dpl_inertial, 'mass')
        dpl_inertial_mass.set('value', '%f' % m)
        dpl_inertial_inertia = gfg.SubElement(dpl_inertial, 'inertia')
        dpl_inertial_inertia.set('ixx', '%f' % ixx)
        dpl_inertial_inertia.set('ixy', '0.0')
        dpl_inertial_inertia.set('ixz', '0.0')
        dpl_inertial_inertia.set('iyy', '%f' % iyy)
        dpl_inertial_inertia.set('iyz', '0.0')
        dpl_inertial_inertia.set('izz', '%f' % izz)
        
        door_joint = gfg.SubElement(self.root_urdf, 'joint')
        door_joint.set('name', self.door_joint_name)
        door_joint.set('type', 'revolute')

        j0_origin = gfg.SubElement(door_joint, 'origin')
        r, p, y = euler_from_matrix(self.T_A_O)
        j0_origin.set('xyz', '{} {} {}'.format(self.T_A_O[0, 3], self.T_A_O[1,3],  self.T_A_O[2,3]))
        j0_origin.set('rpy', '%f %f %f' % (r, p, y + self.axis_pos*self.initial_angle_rad))
        j0_axis = gfg.SubElement(door_joint, 'axis')
        j0_axis.set('xyz', '0 0 1')
        j0_parent = gfg.SubElement(door_joint, 'parent')
        j0_parent.set('link', base_link_name)
        j0_child = gfg.SubElement(door_joint, 'child')
        j0_child.set('link', door_panel_link_name)
        j0_limit = gfg.SubElement(door_joint, 'limit')
        j0_limit.set('effort', '50')

        if axis_pos == 1:
            j0_limit.set('lower', '0')
            j0_limit.set('upper', '{}'.format(pi/2. + np.deg2rad(5.)))
        else:
            j0_limit.set('lower', '{}'.format(-pi/2. - np.deg2rad(5.)))
            j0_limit.set('upper', '0')

        j0_limit.set('velocity', '10')
        j0_dynamics = gfg.SubElement(door_joint, 'dynamics')
        j0_dynamics.set('friction', '0.02')
        j0_dynamics.set('damping', '0.05')

        gazebo_ = gfg.SubElement(self.root_urdf, 'gazebo')
        gazebo_.set('reference', base_link_name)
        gazebo_material = gfg.SubElement(gazebo_, 'material')
        gazebo_material.text = 'Gazebo/Grey'

        gazebo__ = gfg.SubElement(self.root_urdf, 'gazebo')
        gazebo__.set('reference', door_panel_link_name)
        gazebo_material_ = gfg.SubElement(gazebo__, 'material')
        gazebo_material_.text = 'Gazebo/Grey'


        if self.has_handle:
            
            handle_link = gfg.SubElement(self.root_urdf, 'link')
            handle_link.set('name', self.handle_link_name)
            
            handle_visual = gfg.SubElement(handle_link, 'visual')
            h_visual_origin = gfg.SubElement(handle_visual, 'origin')
            h_visual_origin.set('xyz', '0.0 0.0 0.0')
            h_visual_origin.set('rpy', '0.0 0.0 0.0')

            h_visual_geom = gfg.SubElement(handle_visual, 'geometry')
            h_visual_geom_cyl = gfg.SubElement(h_visual_geom, 'cylinder')
            h_visual_geom_cyl.set('length', '{}'.format(self.handle_length))
            h_visual_geom_cyl.set('radius', '{}'.format(self.handle_radius))

            handle_collision = gfg.SubElement(handle_link, 'collision')
            handle_collision.set('name', self.handle_link_name+'_collision_0')
            h_collision_origin = gfg.SubElement(handle_collision, 'origin')
            h_collision_origin.set('xyz', '0.0 0.0 0.0')
            h_collision_origin.set('rpy', '0.0 0.0 0.0')

            h_collision_geom = gfg.SubElement(handle_collision, 'geometry')
            h_collision_geom_cyl = gfg.SubElement(h_collision_geom, 'cylinder')
            h_collision_geom_cyl.set('length', '{}'.format(self.handle_length))
            h_collision_geom_cyl.set('radius', '{}'.format(self.handle_radius))

            handle_base0_visual = gfg.SubElement(handle_link, 'visual')
            hb0_visual_origin = gfg.SubElement(handle_base0_visual, 'origin')
            hb0_visual_origin.set('xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, self.handle_length/4.))
            hb0_visual_origin.set('rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))
            
            hb0_visual_geom = gfg.SubElement(handle_base0_visual, 'geometry')
            hb0_visual_geom_cyl = gfg.SubElement(hb0_visual_geom, 'cylinder')
            hb0_visual_geom_cyl.set('length', '{}'.format(self.handle_height))
            hb0_visual_geom_cyl.set('radius', '{}'.format(self.handle_base_radius))

            handle_base0_collision = gfg.SubElement(handle_link, 'collision')
            handle_base0_collision.set('name', self.handle_link_name+'_collision_1')
            hb0_collision_origin = gfg.SubElement(handle_base0_collision, 'origin')
            hb0_collision_origin.set('xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, self.handle_length/4.))
            hb0_collision_origin.set('rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))
            
            hb0_collision_geom = gfg.SubElement(handle_base0_collision, 'geometry')
            hb0_collision_geom_cyl = gfg.SubElement(hb0_collision_geom, 'cylinder')
            hb0_collision_geom_cyl.set('length', '{}'.format(self.handle_height/2.))
            hb0_collision_geom_cyl.set('radius', '{}'.format(self.handle_base_radius))

            handle_base1_visual = gfg.SubElement(handle_link, 'visual')
            hb1_visual_origin = gfg.SubElement(handle_base1_visual, 'origin')
            hb1_visual_origin.set('xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, -self.handle_length/4.))
            hb1_visual_origin.set('rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))
            
            hb1_visual_geom = gfg.SubElement(handle_base1_visual, 'geometry')
            hb1_visual_geom_cyl = gfg.SubElement(hb1_visual_geom, 'cylinder')
            hb1_visual_geom_cyl.set('length', '{}'.format(self.handle_height))
            hb1_visual_geom_cyl.set('radius', '{}'.format(self.handle_base_radius))

            handle_base1_collision = gfg.SubElement(handle_link, 'collision')
            handle_base1_collision.set('name', self.handle_link_name+'_collision_2')
            hb1_collision_origin = gfg.SubElement(handle_base1_collision, 'origin')
            hb1_collision_origin.set('xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, -self.handle_length/4.))
            hb1_collision_origin.set('rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))
            
            hb1_collision_geom = gfg.SubElement(handle_base1_collision, 'geometry')
            hb1_collision_geom_cyl = gfg.SubElement(hb1_collision_geom, 'cylinder')
            hb1_collision_geom_cyl.set('length', '{}'.format(self.handle_height))
            hb1_collision_geom_cyl.set('radius', '{}'.format(self.handle_base_radius))

            h_inertial = gfg.SubElement(handle_link, 'inertial')
            m_h = 0.05
            ixx_h = 1/12.*m_h*((2*self.handle_radius)**2 + (self.handle_length)**2)
            iyy_h = 1/12.*m_h*((self.handle_radius+self.handle_height)**2 + (self.handle_length)**2)
            izz_h = 1/12.*m_h*((2*self.handle_radius)**2 + (self.handle_radius+self.handle_height)**2)

            h_inertial_mass = gfg.SubElement(h_inertial, 'mass')
            h_inertial_mass.set('value', '%f' % m_h)
            h_inertial_inertia = gfg.SubElement(h_inertial, 'inertia')
            h_inertial_inertia.set('ixx', '%f' % ixx_h)
            h_inertial_inertia.set('ixy', '0.0')
            h_inertial_inertia.set('ixz', '0.0')
            h_inertial_inertia.set('iyy', '%f' % iyy_h)
            h_inertial_inertia.set('iyz', '0.0')
            h_inertial_inertia.set('izz', '%f' % izz_h)

            handle_joint = gfg.SubElement(self.root_urdf, 'joint')
            handle_joint.set('name', self.handle_joint_name)
            handle_joint.set('type', 'fixed')
            h_j_origin = gfg.SubElement(handle_joint, 'origin')
            h_j_origin.set('xyz', '{} {} {}'.format(self.T_H_A[0,3], self.T_H_A[1,3], self.T_H_A[2,3]))
            
            Tz_ = np.eye(4)
            Tz_[:3, :3] = rot_z(np.pi/2 - axis_pos*np.pi/2)
            
            h_j_origin.set('rpy', '%f %f %f' % euler_from_matrix(Tz_))
            h_j_parent = gfg.SubElement(handle_joint, 'parent')
            h_j_parent.set('link', door_panel_link_name)
            h_j_child = gfg.SubElement(handle_joint, 'child')
            h_j_child.set('link', self.handle_link_name)
 

        xmlstr = minidom.parseString(gfg.tostring(self.root_urdf)).toprettyxml(indent='\t')

        if save_path:
            if not os.path.exists(os.path.dirname(save_path)):
                os.makedirs(os.path.dirname(save_path))
            with open (save_path, 'w') as f:
                f.write(xmlstr)

        return xmlstr


    def change_door_angle(self, angle_deg):
        self.current_angle_state_deg = angle_deg
        angle_rad = np.radians(angle_deg)

        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(angle_rad)
        self.T_A_O = self.T_A_O_init @ Tz
        return self.T_A_O


