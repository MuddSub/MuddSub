#!/usr/bin/env python
import rospy
import xml.etree.ElementTree as ET

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
        rospy.set_param("robot_name", self.robotName)
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

    for link in urdfRoot.findall("link"):
        if link.attrib["name"] == rospy.get_param("~base_link"):
            inertial = link.find("inertial")
            properties.mass = float(inertial.find("mass").attrib['value'])
            properties.cg = {}
            for i in "xyz":
                properties.cg[i] = float(inertial.find("origin").attrib[i])

            properties.inertia = {}
            for i in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                properties.inertia[i] = float(inertial.find("inertia").attrib[i])

            d =  link.find("underwater_dynamics")
            properties.buoyancy = float(d.find("buoyancy").attrib['value'])
            properties.cb = {}
            for i in "xyz":
                properties.cb[i] = float(d.find("center_of_buoyancy").attrib[i])

            properties.linearDamping = {}
            properties.addedMass = {}
            for i in "xyzkmn":
                properties.linearDamping[i] = float(d.find("linear_damping_coeffs").attrib[i])
                properties.addedMass[i] = float(d.find("added_mass").attrib[i])
            properties.rate = float(d.find("rate").attrib["value"])

    properties.dump()
    properties.setParams()
