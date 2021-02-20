#!/usr/bin/env python3

import sys
print("HERE", sys.version)
import rospy
import xml.etree.ElementTree as ET

## Loads parameters from description into the ROS parameter server
class Properties:
    def __init__(self):
        self.cg = None
        self.cb = None
        self.mass = None
        self.buoyancy = None
        self.addedMass = None
        self.inertia = None
        self.linearDamping = None
        self.rate = None

        self.robotName = None

    def dump(self):
        print("===Properties===")
        print("Name: ", self.robotName)
        print("CG: ", self.cg)
        print("CB: ", self.cb)
        print("Mass: ", self.mass)
        print("Buoyancy: ", self.buoyancy)
        print("Added Mass" ,self.addedMass)
        print("Inertia" ,self.inertia)
        print("linearDamping" ,self.linearDamping)
        print("rate" ,self.rate)
        print("===EndProperties===")

    def setParams(self):
        paramRoot = "dynamics/"+self.robotName+"/"
        rospy.set_param(paramRoot + "center_of_gravity", self.cg)
        rospy.set_param(paramRoot + "center_of_buoyancy", self.cb)
        rospy.set_param(paramRoot + "mass", self.mass)
        rospy.set_param(paramRoot + "buoyancy", self.buoyancy)
        rospy.set_param(paramRoot + "added_mass", self.addedMass)
        rospy.set_param(paramRoot + "inertia", self.inertia)
        rospy.set_param(paramRoot + "linear_damping_coeffs", self.linearDamping)
        rospy.set_param(paramRoot + "rate", self.rate)
        rospy.set_param("dynamics_ready", True)

if __name__ == '__main__':
    # Load the URDF into a parameter (just as raw text) then set the private param
    rospy.init_node("XMLParser", anonymous=True)

    urdfName = rospy.get_param("~urdf_name")
    urdf = rospy.get_param(urdfName)
    urdfRoot = ET.fromstring(urdf)

    properties = Properties()

    properties.robotName = urdfRoot.attrib['name']

    print("Base Link: ", rospy.get_param("~base_link"))

    baseLink = rospy.get_param("robot_name") + "/" + rospy.get_param("~base_link")

    for link in urdfRoot.findall("link"):
        if link.attrib["name"] == baseLink:
            inertial = link.find("inertial")
            properties.mass = float(inertial.find("mass").attrib['value'])


            properties.cg = {}
            cb = inertial.find("origin").attrib["xyz"].split()
            for i in range(3):
                s = "xyz"[i]
                properties.cg[s] = cb[i]

            properties.inertia = {}
            for i in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                properties.inertia[i] = float(inertial.find("inertia").attrib[i])

    for gazebo in urdfRoot.findall("gazebo"):
        for plugin in gazebo.findall("plugin"):
            if plugin.attrib["name"] == "uuv_plugin":
                density = float(plugin.find("fluid_density").text)

                for link in plugin.findall("link"):
                    if link.attrib["name"] == baseLink:
                        volume = float(link.find("volume").text)
                        gravity = .98
                        properties.buoyancy = volume * density * gravity

                        properties.cb = {}
                        cb = link.find("center_of_buoyancy").text.split()
                        for i in range(3):
                            s = "xyz"[i]
                            properties.cb[s] = float(cb[i])

                        hydrodynamics = link.find("hydrodynamic_model")

                        linearDamping = hydrodynamics.find("linear_damping").text.split()
                        properties.linearDamping = {}
                        for i in range(6):
                            s = "xyzkmn"[i]
                            properties.linearDamping[s] = float(linearDamping[i])

                        properties.addedMass = {}
                        addedMass = hydrodynamics.find("added_mass").text.split()
                        if len(addedMass) == 6:
                            indices = list(range(6))
                        else:
                            indices = [i*6 + i for i in range(6)]

                        for i in range(6):
                            index = indices[i]
                            s = "xyzkmn"[i]
                            properties.addedMass[s] = addedMass[index]
                        break




    properties.rate = 20
    # for i in "xyzkmn":
    #     properties.linearDamping[i] = float(d.find("linear_damping_coeffs").attrib[i])
    #     properties.addedMass[i] = float(d.find("added_mass").attrib[i])
    # properties.rate = float(d.find("rate").attrib["value"])

    properties.dump()
    properties.setParams()
