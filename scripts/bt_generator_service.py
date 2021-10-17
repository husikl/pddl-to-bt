from typing import no_type_check_decorator
from lxml.builder import E
from lxml import etree
import rospy
import numpy as np
import actionlib
from behaviortree_ros.msg import BTExecutorAction, BTExecutorGoal
from behaviortree_ros.srv import GetBT, GetBTResponse
from geometry_msgs.msg import PoseStamped, Point
from youbot_projected_gradient.msg import object_pose
from collections import deque


parallelNodeCreated = False
path_check_xml = 'IsPathValid'
print_xml = '<PrintValue message="some_message"/>'
# some_xml_data = '<BehaviorTree ID="MoveRobot"><Fallback><SequenceStar><MoveBase       goal="{target}"/><SetBlackboard output_key="output" value="mission accomplished" /></SequenceStar></Fallback></BehaviorTree>'
# element = etree.fromstring(some_xml_data)

move_base_xml = '<MoveBase server_name="mecanum/move_base" x="0" y ="0" frame="mecanum/map"/>'

# pick_xml = '<PickProjected server_name="bt_pick" id="else" pose_x="0.0" pose_y="-1.0" pose_z="0.1" /> '
# place_xml = '<PlaceProjected server_name="bt_pick" id="else" pose_x="0.0" pose_y="-1.0" pose_z="0.1" />'
# approach_xml = '<ApproachProjected server_name="bt_approach" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
# detect_xml = '<DetectIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
# request_assist_xml = '<AssistIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'

pick_xml = '<PickProjected server_name="bt_pick" id="else" pose = "0.0;0.0;0.0" input="{approachedResult}" output="{picked}" />'
place_xml = '<PlaceProjected server_name="/bt_place" id="none" location="none" at="0.0; 0.0; 0.0" />'
approach_xml = '<ApproachProjected server_name="bt_approach" id="else" pose="0.0; 0.0; 0.0" output="{approachedResult}" />'
search_ipp_xml = '<ExploreIPP service_name="/youbot/self_explore" trajectory="{traj}"  />'

detect_xml = '<DetectIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
request_assist_xml = '<SupportReceiver server_name="support_receiver" type="check_path" path="{traj}" at="{location}" />'


is_path_valid_xml ='<IsPathValid service_name="gridmap/trajEntropy" pose="0.0;0.0;0.0" segment="{traj}" />'
is_path_coll_free_xml ='<IsPathCollFree service_name="coll_check_srv" pose="0.0;0.0;0.0"/>'
is_grasped_xml ='<IsGraspedObj service_name="gripper_state_server" id="None" />'
is_hand_xml ='<IsHandFree service_name="gripper_state_server" id="None" />'
is_robot_close_xml ='<IsRobotCloseTo service_name="is_robot_close_to" id="None" pose="0.0;0.0;0.0" />'

is_obj_at_xml ='<IsObjectAt  id="None" location="None" />'
is_obj_found_xml ='<IsObjectAt  object="None" pose="None" />'

class ActionTemp:
    __slots__ = ['precond', 'action', 'effects', 'xml', 'cos']

    def __init__(self,xml, p, e, a, cos):
        self.precond = p
        self.action =  a
        self.xml = xml
        self.effects =  e
        self.cos = cos

    def to_string(self, property):
        s =""
        for p in property :
            
            s += str(p) 
        return s


class Obstacle:
    __slots__ = ['name', 'position']
    
    def __init__(self, name, position):
        self.name = name
        self.position = position

class Location:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected

class Target:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected

class Belief:
    __slots__ = ['distribution']
    
    def __init__(self, distribution):
        self.distribution = distribution

    def update(self, belief):
        return belief


class Constraint:
    __slots__ = ['type', 'xml', 'params']
    
    def __init__(self, type, xml, params):
        self.type = type
        self.xml = xml        
        self.params = params
    def __copy__(self):
        cls = self.__class__
        newobject = cls.__new__(cls)
        newobject.__dict__.update(self.__dict__)
        newobject.params = self.params
        newobject.xml = self.xml
        newobject.type = self.type
        return newobject

class ConstraintOperativeSubspace:
    def __init__(self, variables, constraints):
        self.variables = variables
        self.constraints = constraints



        
is_obj_detected = Constraint('is_object_at', is_obj_at_xml,{'object': 0, 'detected': 0}) #for detect or so called get observation action 
is_object_at = Constraint('is_object_at', is_obj_at_xml,{'object': 0, 'at': 0}) 
is_robot_close_to = Constraint('is_robot_close_to',is_robot_close_xml, {'robot': 0, 'to': 0})
is_grasped = Constraint('is_grasped',is_grasped_xml, {'object': 0, 'hand':0})
is_hand_free = Constraint('is_hand_free',is_hand_xml, {'empty': 0, 'hand':0})
is_path_valid = Constraint('is_path_valid',is_path_valid_xml, {'path': 0, 'object': 0})
is_path_coll_free = Constraint('is_path_coll_free',is_path_coll_free_xml, {'path': 0, 'object': 0})

constraints= []
constraints.append(is_obj_detected)
constraints.append(is_object_at)
constraints.append(is_robot_close_to)
constraints.append(is_hand_free)
constraints.append(is_grasped)

constraints.append(is_path_valid)

# approach_to = ActionTemp(approach_xml,[is_path_coll_free,is_path_valid],['robot','to'],'ApproachProjected', 'pose2d')
approach_to = ActionTemp(approach_xml,[is_path_valid],['robot','to'],'ApproachProjected', 'pose2d')
detect_obj = ActionTemp(detect_xml,[],['object','found'],'DetectIPP','o')
assist_detect_obj = ActionTemp(request_assist_xml,[],['path'],'SupportReceiver','e' )
pick_obj = ActionTemp(pick_xml,[is_hand_free,is_robot_close_to],['object','hand'],'PickProjected', 'o')

place_obj = ActionTemp(place_xml,[is_grasped,is_robot_close_to],['object','at'],'PlaceProjected', 'at')

# drop_obj = ActionTemp(place_xml,[],['hand','empty'],'drop_obj', {'at': 0})
explore_path = ActionTemp(search_ipp_xml,[],['path'],'ExploreIPP','e')

actions = []
actions.append(approach_to)
actions.append(detect_obj)
actions.append(assist_detect_obj)
actions.append(pick_obj)
actions.append(place_obj)

# actions.append(drop_obj)
actions.append(explore_path)

### -----------------------------------------------------------------------------------------------------

def get_pose_by_id(object_id):
        
    check_pose = object_pose()
    check_pose.coordinate.x = 1.0
    check_pose.coordinate.x = 2.0
    check_pose.coordinate.x = 3.0

    return check_pose

globalObjs = {
  "storage": "0.5;-1.0;0.0",
  "target": "0.5;-3.0;0.0",
}

dict = {}

def getXmlFromConditionString(goal_condition):
    global constraints
    goalCondition = etree.fromstring(goal_condition)
    for c in constraints :
        element = etree.fromstring(c.xml)
        if goalCondition.tag == element.tag :
            condition = c
    
    condition.xml = etree.tostring(goalCondition, encoding="unicode", pretty_print=True)
    return goalCondition, condition

def getConditionFromXml(xml):
    global constraints
    condition = None
    # print("xml = ", xml)
    for c in constraints :
        element = etree.fromstring(c.xml)
        if xml.tag == element.tag :
            condition = c
            condition.xml = etree.tostring(xml, encoding="unicode", pretty_print=True)
            # print("condition found type = ", condition.type)
    return condition

def createInitialTree(goal_condition):
    root = etree.Element("root")
    bt = etree.Element("BehaviorTree")
    fallback = etree.Element("Fallback")
    root.append(bt)   
    
    goalCondition, condition = getXmlFromConditionString(goal_condition)
    fallback.append(goalCondition)
    bt.append(fallback)
    # print("here")
    sub_bt = get_subtree(condition)
    fallback.append(sub_bt)
    tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    print(tree)
    root.clear()
    return tree

def updateTree(tree, goal_condition):
    rospy.loginfo("update for goal condition = %s", goal_condition)
    root = etree.fromstring(tree)
    global dict
    # tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    # foundElement = root.findall(".//%s" % str(goal_condition))
    foundElement = []
    # for child in root.getroot().iterchildren("*"):
    #     print(child.tag)
    #     if child.tag == goal_condition:
    #         foundElement.append(child)
    queue = deque([root])
    while queue:
        el = queue.popleft()  # pop next element
        queue.extend(el)      # append its children
        # print(el.tag)
        if el.tag == goal_condition:
            foundElement.append(el)
    
    found = None
    # print("##### = ", foundElement)
    # return
    for i in foundElement :
        if i.tag == "IsRobotCloseTo"  :
            dict[i] = [i, i.tag, i.attrib['pose']]
            found = i
        elif i.tag == "IsPathValid" :
            dict[i] = [i, i.tag]
            found = i
            print ("here found IsPathValid")
        else :
            if 'id' in i.attrib :
                dict[i.tag] = i.attrib['id']
            else :
                dict[i] = [i, i.tag]
            found = i
            break
    if found.tag == "IsRobotCloseTo"  :
        for key in dict :
            if dict[key][1] == "IsRobotCloseTo" :
                found = dict[key][0]
    # print("found= ", found)
    if found is None :
        print("not found ...")
        root.clear()
        return tree
    index = found.getparent().index(found)
    # print('index= ', index)
    sequenceStar = etree.Element("Sequence") 
    fallback = etree.Element("Fallback")
    parent = found.getparent()
    found.getparent().remove(found)
    fallback.append(found)
    # print("[]: ", found)
    condition = getConditionFromXml(found)
    if not condition :
        root.clear()
        return tree
    sub_bt = get_subtree(condition)
    fallback.append(sub_bt)
    sequenceStar.append(fallback)
    parent.insert(index, sequenceStar)
    tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    root.clear()
    return tree
    



############################################################################################################
def get_subtree(constraint):
        global parallelNodeCreated
        global actions
        bt = None
        # print(constraint.type)
        acts = []
        for action in actions:
            # print('Trying with action', action.name, 'Effects', action.effects, 'Condition fluents', fluent.parameters_dict.keys())
            if set(action.effects).issubset(set(constraint.params.keys())):
                
                
                # print('The action ', action.name, ' can hold ', fluent.name)
                bt = etree.Element("Sequence")
                action_xml = set_action_values_from_condition(action, constraint)
                for c in action.precond:
                    c_xml = set_condition_values_from_action(c, action_xml)
                    bt.append(c_xml)
                
                bt.append(action_xml)
                acts.append(action_xml)
        if len(acts) > 1 and not parallelNodeCreated:
            if constraint.type == "is_path_valid" :
                tree =  etree.Element("Parallel")
                print("setting parallel");
                for t in acts :
                    tree.append(t)
                tree.set("success_threshold","1")
                tree.set("failure_threshold","1")
                parallelNodeCreated = True
                bt.clear()
                return tree
            
            tree =  etree.Element("Fallback")
            for t in acts :
                tree.append(t)
            return tree
        if bt is None:
            raise Exception('Cannot find action with effects', constraint.params.keys())
        return bt


def set_action_values_from_condition(action, condition):
    global globalObjs
    action_xml = etree.fromstring(action.xml)
    c = etree.fromstring(condition.xml)
    if action.cos == "pose2d" :
        if 'id' in c.attrib :
            action_xml.attrib['id'] = c.attrib['id']
            action_xml.attrib['pose'] = globalObjs[action_xml.attrib['id']]

    elif action.cos == "o" :
        if 'id' in c.attrib :
            action_xml.attrib['id'] = c.attrib['id']


    elif action.cos == "at" :
        action_xml.attrib['location'] = c.attrib['location']
        action_xml.attrib['id'] = c.attrib['id']
        action_xml.attrib['at'] = globalObjs["storage"]

    return action_xml

def set_condition_values_from_action(condition, action_xml):
    global actions
    c = etree.fromstring(condition.xml)
    action = None
    for a in actions :
        if a.action == action_xml.tag :
            action = a
    # print("action found = ", action.action)
    if action.cos == "at" :

        if 'id' in c.attrib:
            c.attrib['id'] = action_xml.attrib['id']
        if 'pose' in c.attrib and 'id' in c.attrib:
            c.attrib['id'] = action_xml.attrib['location']
            c.attrib['pose'] = action_xml.attrib['at']
            # if action_xml.attrib['location'] == "storage":
            #     # print("true,", " ", action_xml.attrib['id'], " ",  globalObjs[action_xml.attrib['id']])
            #     c.attrib['pose'] = globalObjs[action_xml.attrib['location']]
    else :
        print("inside else") 
        if 'id' in c.attrib:
            c.attrib['id'] = action_xml.attrib['id']
            print("set id")
        if 'pose' in c.attrib:
            c.attrib['pose'] = action_xml.attrib['pose']
            if action_xml.attrib['id'] == "storage" or action_xml.attrib['id'] == "target":
                # print("true,", " ", action_xml.attrib['id'], " ",  globalObjs[action_xml.attrib['id']])
                c.attrib['pose'] = globalObjs[action_xml.attrib['id']]
                print("set pose")
    

    return c

############################################################################################################


def create_bt(req):
    if req.type == str("new"):
        tree = GetBTResponse(createInitialTree(req.goal_condition))
    if req.type == str("update"):
        tree = GetBTResponse(updateTree(req.bt, req.goal_condition))

    return tree

def create_bts_server():
    rospy.init_node('bt_generator_service')
    s = rospy.Service('get_bt_service', GetBT, create_bt)
    print("Ready to generate behavior trees")
    rospy.spin()


if __name__ == '__main__':
    try:
        create_bts_server()
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)