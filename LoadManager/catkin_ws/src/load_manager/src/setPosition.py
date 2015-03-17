import sys
from lxml import etree

PATH = "/opt/ros/hydro/share/turtlebot_navigation/launch/"
FILE = "amcl_demo_edited.launch"

tree = etree.parse(PATH + FILE)
root = tree.getroot()

for elem in root.findall("arg"):
    if elem.attrib["name"] == "initial_pose_x":
        elem.attrib["default"] = sys.argv[1]
    if elem.attrib["name"] == "initial_pose_y":
        elem.attrib["default"] = sys.argv[2]
    if elem.attrib["name"] == "initial_pose_a":
        elem.attrib["default"] = sys.argv[3]

        
with open(PATH + FILE, 'w') as file_handle:
    file_handle.write(etree.tostring(tree, pretty_print=True, encoding='utf8'))
