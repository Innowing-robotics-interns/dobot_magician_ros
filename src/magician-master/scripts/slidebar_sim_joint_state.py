#!/usr/bin/env python

########################################################################################
# This node shows a slidebar GUI for controlling the robot arm's joint states (sim).
########################################################################################


import rospy
import random
import wx
import wx.lib.newevent
import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
from threading import Thread

RANGE = 10000

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class JointStatePublisher():
    def __init__(self):
        description = get_param('robot_description')
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        use_mimic = get_param('use_mimic_tags', True)
        use_small = get_param('use_smallest_joint_limits', True)

        self.zeros = get_param("zeros")

        pub_def_positions = get_param("publish_default_positions", True)
        pub_def_vels = get_param("publish_default_velocities", False)
        pub_def_efforts = get_param("publish_default_efforts", False)

        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if use_small and len(safety_tags)==1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if use_mimic and len(mimic_tags)==1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval}
                if pub_def_positions:
                    joint['position'] = zeroval
                if pub_def_vels:
                    joint['velocity'] = 0.0
                if pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint


        use_gui = get_param("use_gui", False)

        if use_gui:
            self.app = wx.App()
            self.gui = JointStatePublisherGui("Joint State Publisher", self)
            self.gui.Show()
        else:
            self.gui = None

        source_list = get_param("source_list", [])
        self.sources = []
        for source in source_list:
            self.sources.append(rospy.Subscriber(source, JointState, self.source_cb))

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.gui is not None:
            # post an event here instead of directly calling the update_sliders method, to switch to the wx thread
            wx.PostEvent(self.gui.GetEventHandler(), self.gui.UpdateSlidersEvent())


    def loop(self):
        hz = get_param("rate", 10) # 10hz
        r = rospy.Rate(hz)

        delta = get_param("delta", 0.0)

        # Publish Joint States
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()

            if delta > 0:
                self.update(delta)

            # Initialize msg.position, msg.velocity, and msg.effort.
            has_position = len(self.dependent_joints.items()) > 0
            has_velocity = False
            has_effort = False
            for (name,joint) in self.free_joints.items():
                if not has_position and 'position' in joint:
                    has_position = True
                if not has_velocity and 'velocity' in joint:
                    has_velocity = True
                if not has_effort and 'effort' in joint:
                    has_effort = True
            num_joints = (len(self.free_joints.items()) +
                          len(self.dependent_joints.items()))
            if has_position:
                msg.position = num_joints * [0.0]
            if has_velocity:
                msg.velocity = num_joints * [0.0]
            if has_effort:
                msg.effort = num_joints * [0.0]


            for i, name in enumerate(self.joint_list):
                msg.name.append(str(name))
                joint = None
                arm_id_1 = 0
                arm_id_2 = 0
                ##########
                #if name == 'joint_2':
                #    arm_id_1 = i
                #if name == 'joint_5':
                #    arm_id_2 = i
                ##########
                # Add Free Joint
                if name in self.free_joints:
                    joint = self.free_joints[name]
                    factor = 1
                    offset = 0
                # Add Dependent Joint
                elif name in self.dependent_joints:
                    param = self.dependent_joints[name]
                    parent = param['parent']
                    joint = self.free_joints[parent]
                    factor = param.get('factor', 1)
                    offset = param.get('offset', 0)

                if has_position and 'position' in joint:
                    msg.position[i] = joint['position'] * factor + offset
                if has_velocity and 'velocity' in joint:
                    msg.velocity[i] = joint['velocity'] * factor
                if has_effort and 'effort' in joint:
                    msg.effort[i] = joint['effort']
                ##########
                if name == 'joint_6':
                    #msg.position[i] = msg.position[arm_id_2] - msg.position[arm_id_1]
                    msg.position[3] = msg.position[2] - msg.position[1]

                if name == 'joint_7':
                    msg.position[4] = msg.position[3]
                ##########

            self.pub.publish(msg)
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass

    def update(self, delta):
        for name, joint in self.free_joints.iteritems():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward

class JointStatePublisherGui(wx.Frame):
    def __init__(self, title, jsp):
        wx.Frame.__init__(self, None, -1, title, (-1, -1));
        self.jsp = jsp
        self.joint_map = {}
        panel = wx.ScrolledWindow(self, wx.ID_ANY, style= wx.VSCROLL);
        box = wx.BoxSizer(wx.VERTICAL)
        font = wx.Font(9, wx.SWISS, wx.NORMAL, wx.BOLD)

        ### Sliders ###
        for name in self.jsp.joint_list:
            if name not in self.jsp.free_joints:
                continue
            ########
            if name == 'joint_6':
                continue
            ########
            joint = self.jsp.free_joints[name]

            if joint['min'] == joint['max']:
                continue


            row = wx.GridSizer(1,2,1,1)
            label = wx.StaticText(panel, -1, name)
            label.SetFont(font)
            row.Add(label, 1, wx.ALIGN_CENTER_VERTICAL)

            display = wx.TextCtrl (panel, value=str(0),
                        style=wx.TE_READONLY | wx.ALIGN_RIGHT)

            row.Add(display, flag= wx.ALIGN_RIGHT| wx.ALIGN_CENTER_VERTICAL)
            box.Add(row, 1, wx.EXPAND)
            slider = wx.Slider(panel, -1, RANGE/2, 0, RANGE,
                        style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL)
            slider.SetFont(font)
            box.Add(slider, 1, wx.EXPAND)

            self.joint_map[name] = {'slidervalue':0, 'display':display,
                                    'slider':slider, 'joint':joint}

        self.UpdateSlidersEvent, self.EVT_UPDATESLIDERS = wx.lib.newevent.NewEvent()
        self.Bind(self.EVT_UPDATESLIDERS, self.updateSliders)

        ### Buttons ###
        self.randbutton = wx.Button(panel, 1, 'Randomize')
        self.ctrbutton = wx.Button(panel, 2, 'Center')
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)

        wx.EVT_BUTTON(self, 1, self.randomize_event)
        wx.EVT_BUTTON(self, 2, self.center_event)

        box.Add(self.randbutton, 0, wx.EXPAND)
        box.Add(self.ctrbutton, 1, wx.EXPAND)

        panel.SetSizer(box)
        self.center()
        box.Fit(self)
        panel.SetScrollRate(0,slider.GetSize().GetHeight()+row.GetSize().GetHeight())
        self.update_values()


    def update_values(self):
        for (name,joint_info) in self.joint_map.items():
            purevalue = joint_info['slidervalue']
            joint = joint_info['joint']
            value = self.sliderToValue(purevalue, joint)
            joint['position'] = value
        self.update_sliders()

    def updateSliders(self, event):
        self.update_sliders()

    def update_sliders(self):
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['position'],
                                                           joint)
            joint_info['slider'].SetValue(joint_info['slidervalue'])
            joint_info['display'].SetValue("%.2f"%joint['position'])

    def center_event(self, event):
        self.center()

    def center(self):
        rospy.loginfo("Centering")
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['zero'], joint)
        self.update_values()

    def randomize_event(self, event):
        self.randomize()

    def randomize(self):
        rospy.loginfo("Randomizing")
        for (name,joint_info) in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(random.uniform(joint['min'], joint['max']), joint)
        self.update_values()


    def sliderUpdate(self, event):
        for (name,joint_info) in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].GetValue()
        self.update_values()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue


if __name__ == '__main__':
    try:
        rospy.init_node('magician_fake_joint_state_publisher')
        jsp = JointStatePublisher()

        if jsp.gui is None:
            jsp.loop()
        else:
            Thread(target=jsp.loop).start()
            jsp.app.MainLoop()

    except rospy.ROSInterruptException: pass
