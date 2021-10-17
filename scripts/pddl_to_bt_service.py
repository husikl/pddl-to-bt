# from pddlstream.algorithms.satisfaction import obj_from_existential_expression
# from examples.table_belief.hus_service import objects_info
from sys import dont_write_bytecode
from typing import Sequence
# from typing import no_type_check_decorator
from lxml.builder import E
from lxml import etree
import rospy
# import numpy as np
# import actionlib
from behaviortree_ros.msg import BTExecutorAction, BTExecutorGoal
from behaviortree_ros.srv import GetBT, GetBTResponse, SendTree, GetObjectsInfo
# from geometry_msgs.msg import PoseStamped, Point
from youbot_projected_gradient.msg import object_pose
from collections import deque
# from collections import OrderedDict
from itertools import tee
import copy
import threading
import time

global objectsInfo
objectsInfo = None
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

pick_xml = '<PickProjected server_name="bt_pick" id="else" on="" pose = "0.0;0.0;0.0" />'
place_xml = '<PlaceProjected server_name="/bt_place" id="none" location="none" at="0.0; 0.0; 0.0" />'
approach_xml = '<ApproachProjected server_name="bt_approach" id="else" pose="0.0; 0.0; 0.0" />'
search_ipp_xml = '<ExploreIPP service_name="/youbot/self_explore" trajectory="{traj}"  />'

detect_xml = '<DetectObj service_name="get_observation_srv" id="else" on="" detected="{detRes}" atLoc="{atL}" />'
request_assist_xml = '<SupportReceiver server_name="support_receiver" type="check_path" path="{traj}" at="{location}" />'


is_path_valid_xml ='<IsPathValid service_name="gridmap/trajEntropy" pose="0.0;0.0;0.0" segment="{traj}" />'
is_path_coll_free_xml ='<IsPathCollFree service_name="coll_check_srv" pose="0.0;0.0;0.0"/>'
is_grasped_xml ='<IsGraspedObj service_name="gripper_state_server" id="" />'
is_hand_xml ='<IsHandFree service_name="gripper_state_server" id="" />'
is_robot_close_xml ='<IsRobotCloseTo service_name="is_robot_close_to" location="" pose="0.0;0.0;0.0" />'

is_obj_at_xml ='<IsObjectAt  id="None" location="None" />'
is_obj_detected_xml ='<IsObjDetected service_name="get_observation_srv" id="else" on="" detected="{detRes}" objPose="{atL}" />'

class ActionTemp:
    __slots__ = ['precond', 'name', 'effects', 'xml', 'cos', 'effects_xml']

    def __init__(self,xml, p, e, a, cos, effects_xml):
        self.precond = p
        self.name =  a
        self.xml = xml
        self.effects =  e
        self.cos = cos
        self.effects_xml = effects_xml

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



        
is_obj_detected = Constraint('is_object_detected', is_obj_detected_xml,{'object': 0, 'found': 0}) #for detect or so called get observation action 
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
approach_to = ActionTemp(approach_xml,[is_path_valid],['robot','to'],'approach', 'pose2d', [is_robot_close_to])
detect_obj = ActionTemp(detect_xml,[is_robot_close_to],['object','found'],'detect','on', [is_obj_detected])
register_obj = ActionTemp(detect_xml,[is_robot_close_to],['object','found'],'register','o', [is_obj_detected])
assist_detect_obj = ActionTemp(request_assist_xml,[],['path'],'assist_detect','e', [is_obj_detected] )
pick_obj = ActionTemp(pick_xml,[is_hand_free,is_obj_detected, is_robot_close_to],['object','hand'],'pick', 'ob', [is_grasped])

place_obj = ActionTemp(place_xml,[is_grasped,is_robot_close_to],['object','at'],'place', 'at', [is_object_at])

# drop_obj = ActionTemp(place_xml,[],['hand','empty'],'drop_obj', {'at': 0})
explore_path = ActionTemp(search_ipp_xml,[],['path'],'explore_ipp','e', [is_path_valid])

actions = []
actions.append(approach_to)
actions.append(detect_obj)
actions.append(assist_detect_obj)
actions.append(pick_obj)
actions.append(place_obj)
# actions.append(register_obj)

# actions.append(drop_obj)
actions.append(explore_path)

### -----------------------------------------------------------------------------------------------------

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
    # print(tree)
    root.clear()
    return tree, True

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
        return tree, False
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
        rospy.loginfo("not a condition node ...")
        return tree, False
    sub_bt = get_subtree(condition)
    fallback.append(sub_bt)
    sequenceStar.append(fallback)
    parent.insert(index, sequenceStar)
    tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    root.clear()
    print(tree)
    return tree, True
    



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
    global objectsInfo
    action_xml = etree.fromstring(action.xml)
    c = etree.fromstring(condition.xml)
    if action.cos == "pose2d" and ('id' in c.attrib):
        o_name = c.attrib['id']
        obj1 = None
        print(o_name)
        for o in objectsInfo :
            if o.name == o_name:
                obj1 = o
                break
        print(obj1)
        if 'id' in c.attrib :
            action_xml.attrib['id'] = c.attrib['id']
            action_xml.attrib['pose'] = str(obj1.pose.x) + ';'+ str(obj1.pose.y)+';0'

    elif action.cos == "ob" :
        if 'id' in c.attrib :
            action_xml.attrib['id'] = c.attrib['id']
        # if 'pose' in c.atrib:
        #     action_xml.attrib['pose'] = c.attrib['pose']


    elif action.cos == "at" :
        action_xml.attrib['location'] = c.attrib['location']
        action_xml.attrib['id'] = c.attrib['id']
        # action_xml.attrib['at'] = globalObjs["storage"]

    elif action.cos == "on" :
        action_xml.attrib['on'] = c.attrib['on']
        action_xml.attrib['id'] = c.attrib['id']

    return action_xml

def set_condition_values_from_action(condition, action_xml):
    global actions, objectsInfo
    c = etree.fromstring(condition.xml)
    action = None
    for a in actions :
        xml = etree.fromstring(a.xml)
        if xml.tag == action_xml.tag :
            action = a
    # print("action found = ", action.action)
    if action.cos == "at" :
        if c.tag == str("IsRobotCloseTo"):
            c.attrib['id'] = action_xml.attrib['location']   
            c.attrib['pose'] = action_xml.attrib['at'] 
        else :
            if 'id' in c.attrib:
                c.attrib['id'] = action_xml.attrib['id']
            if 'pose' in c.attrib:
                c.attrib['pose'] = action_xml.attrib['at']

    elif action.cos == "on":
        o_name = action_xml.attrib['on']
        obj = None
        for o in objectsInfo :
            if o.name == o_name :
                obj = o
        if 'id' in c.attrib:
            c.attrib['id'] = o_name
        if 'location' in c.attrib:
            c.attrib['location'] = o_name
        if 'pose' in c.attrib:
            
            c.attrib['pose'] = str(obj.pose.x) + ';'+ str(obj.pose.y)+';0'  
    else :
        print("inside else") 
        if 'id' in c.attrib:
            c.attrib['id'] = action_xml.attrib['id']
        if 'pose' in c.attrib and 'pose' in action_xml.attrib:
            c.attrib['pose'] = action_xml.attrib['pose']
            # if action_xml.attrib['id'] == "storage" or action_xml.attrib['id'] == "target":
                # print("true,", " ", action_xml.attrib['id'], " ",  globalObjs[action_xml.attrib['id']])
                # c.attrib['pose'] = globalObjs[action_xml.attrib['id']]
                # print("set pose")
    

    return c

############################################################################################################

def convert(lst):
    return ' '.join(lst).split()

global converted_tree, actionSubtreeDict
converted_tree = None

actionSubtreeDict = {}

def treeFromPddlPlan(plan):
    tree = None
    get_objects_info()
    reset_all()
    root = etree.Element("root")
    tree = createTreeFromPddl(plan)    
    root.append(tree)   

    test_tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    
    print(test_tree)
    rospy.wait_for_service('/send_tree')
    try:
        rospy.logwarn("sending the tree ...")
        bt = rospy.ServiceProxy('/send_tree', SendTree)
        response = bt(test_tree)
        print(response)
        rospy.logwarn("tree sent?")
        root.clear()

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return test_tree, True

def getEffects(action, a_list):
    global objectsInfo
    objects = []
    effs = []
    action_list = copy.copy(a_list) 
    if action_list[0] == str("place") or action_list[0] == str("pick"):
        last_elt = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                objects.append(o)
                break
        last_elt = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                objects.append(o)
                break
        
        for e in action.effects_xml:
            # print("objects =  ", objects)
            e_xml = etree.fromstring(e.xml)
            if 'id' in e_xml.attrib :
                e_xml.attrib['id'] = objects[0].name
            if 'location' in e_xml.attrib :
                e_xml.attrib['location'] = objects[1].name
            if 'pose' in e_xml.attrib :
                e_xml.attrib['pose'] = str(objects[0].pose.x) + ';'+ str(objects[0].pose.y)+';0' 
            if 'at' in e_xml.attrib :
                e_xml.attrib['at'] = str(objects[1].pose.x) + ';'+ str(objects[1].pose.y)+';0' 

            effs.append(e_xml)

    else :
        # print(action_list)
        last_elt = action_list.pop()
        # print(last_elt)
        for o in objectsInfo:
            if o.name == last_elt :
                obj= o
                # print("obj = ", obj)
                break
        for e in action.effects_xml:
            e_xml = etree.fromstring(e.xml)
            if 'id' in e_xml.attrib :
                e_xml.attrib['id'] = obj.name
            if 'location' in e_xml.attrib :
                e_xml.attrib['location'] = obj.name
            if 'on' in e_xml.attrib :
                e_xml.attrib['on'] = obj.name    
            if 'pose' in e_xml.attrib :
                e_xml.attrib['pose'] = str(obj.pose.x) + ';'+ str(obj.pose.y)+';0' 
            effs.append(e_xml)
    
    
    
    return effs
    raise NotImplementedError

def getActionPrecondtions(action, a_list):
    global objectsInfo
    conds = []
    objects = []
    
    #here the special case should be for pick right ?? leave as todo 
    action_list = copy.copy(a_list) 
    if action_list[0] == str("place") or action_list[0] == str("pick") :
        
        last_elt = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                objects.append(o)
                break
        last_elt = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                objects.append(o)
                break
        
        for c in action.precond:
            c_xml = etree.fromstring(c.xml)
            if 'id' in c_xml.attrib :
                c_xml.attrib['id'] = objects[0].name
            if 'location' in c_xml.attrib :
                c_xml.attrib['location'] = objects[1].name
            if 'on' in c_xml.attrib :
                c_xml.attrib['on'] = objects[1].name
            if 'pose' in c_xml.attrib :
                c_xml.attrib['pose'] = str(objects[1].pose.x) + ';'+ str(objects[1].pose.y)+';0' 


            conds.append(c_xml)

    else :
        last_elt = action_list.pop()
        last_elt2 = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                objects= o
            if o.name == last_elt2 :
                objects2= o
        
        for c in action.precond:
            c_xml = etree.fromstring(c.xml)
            if c.type == str("is_object_detected"):
                c_xml.attrib['id'] = objects.name
                c_xml.attrib['on'] = objects2.name
            
            else :
                if 'id' in c_xml.attrib :
                    # print("-----------------------------")
                    # print(objects, " ", last_elt)
                    # print("-----------------------------")
                    c_xml.attrib['id'] = objects.name
                if 'location' in c_xml.attrib :
                    c_xml.attrib['location'] = objects.name
                if 'pose' in c_xml.attrib :
                    c_xml.attrib['pose'] = str(objects.pose.x) + ';'+ str(objects.pose.y)+';0' 
            conds.append(c_xml)

    return conds
    raise NotImplementedError

def getActionFromPddl(action, a_list):
    # print("getting action for= ",action)
    global objectsInfo
    action_list = copy.copy(a_list) 
    objects = []

    action_xml = etree.fromstring(action.xml)
    if action_list[0] == str("place") or action_list[0] == str("detect") or action_list[0] == str("pick")  :
        last_elt = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                objects.append(o)
                break
        last_elt = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                objects.append(o)
                break
        
        if 'id' in action_xml.attrib :
            action_xml.attrib['id'] = objects[0].name
        if 'pose' in action_xml.attrib :
            action_xml.attrib['pose'] = str(objects[1].pose.x) + ';'+ str(objects[1].pose.y)+';0' 
        if 'location' in action_xml.attrib :
            action_xml.attrib['location'] = objects[1].name
        if 'on' in action_xml.attrib :
            action_xml.attrib['on'] = objects[1].name
        if 'at' in action_xml.attrib :
            action_xml.attrib['at'] = str(objects[1].pose.x) + ';'+ str(objects[1].pose.y)+';0' 

    
    else :
        last_elt = action_list.pop()
        for o in objectsInfo:
            if o.name == last_elt :
                object= o
                break

        if 'id' in action_xml.attrib :
            action_xml.attrib['id'] = object.name
        if 'pose' in action_xml.attrib :
            action_xml.attrib['pose'] = str(object.pose.x) + ';'+ str(object.pose.y)+';0' 
        if 'at' in action_xml.attrib :
            action_xml.attrib['at'] = str(object.pose.x) + ';'+ str(object.pose.y)+';0' 
    return action_xml

    
def createReactiveSubtree(action):
    global actionSubtreeDict
    
    action_list = convert([action])
    print("-------------------")
    print("action list = ", action)
    print("***************************")
    actionTemp = None
    for a in actions:
        # print ("compare ",a.name, " and ", lst[0])
        if a.name == action_list[0]:
            actionTemp = a
            break
    # print("action_list = ", action_list )
    a_xml = getActionFromPddl(actionTemp, action_list)
    fallback = etree.Element("Fallback")
    sequence = etree.Element("Sequence")
    effects = getEffects(actionTemp, action_list)
    precondtions = getActionPrecondtions(actionTemp, action_list)
    for p in precondtions:
        sequence.append(p)
    sequence.append(a_xml)
    for e in effects :
        fallback.append(e)
    fallback.append(sequence)
    actionSubtreeDict[action]= [fallback, precondtions, effects]
    # print('printing subtree ...')
    # f = etree.tostring(fallback, encoding="unicode", pretty_print=True)
    # print(f)
    # print('printing subtree end ..................')
    return fallback




def createTreeFromPddl(plan):
    global converted_tree, actionSubtreeDict
    # l = zip(plan,plan[1:])
    # global converted_tree
    # tree_list = []
    converted_tree = etree.Element("BehaviorTree")
    for a1 in reversed(plan) :
        createReactiveSubtree(a1)
        
    
    
    converted_tree.append(actionSubtreeDict[next(iter(actionSubtreeDict))][0])
    stitchTheTree()
    # sitchSimplest()
    # print("*******************************************************")
    # subtree = etree.tostring(converted_tree, encoding="unicode", pretty_print=True)
    # print(subtree)
    # print("*******************************************************")

    return converted_tree
    # raise NotImplementedError



def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

def stitchTrees(key, value):
    global converted_tree, actionSubtreeDict
    # print("inside stitch trees!!!")
    queue = deque([converted_tree])
    while queue:
        el = queue.popleft()  # pop next element
        queue.extend(el)      # append its children
        # print(el.tag)
        if el.tag == "Sequence":
            for c in el.getchildren():
                for v in value[2]:
                    if c.tag == v.tag:
                        el.replace(c,value[0] )
                        break
            
    

    ## so here t1 is final goal, so it's effect is we keep
    # so we need to check the childs of sequence node in t1 and childs of fallback node in t2!
    # compare the childs and stitch the trees by replacing the node
    
    # raise NotImplementedError

def stitchTheTree():
    global actionSubtreeDict
    print("-----------------------------------")
    # for (key1, value1), (key2, value2) in pairwise(actionSubtreeDict.items()):
    for (key1, value1), (key2, value2) in pairwise(actionSubtreeDict.items()):
        stitchTrees(key1,value2)

def sitchSimplest():
    global actionSubtreeDict, converted_tree
    seq = etree.Element("SequenceStar")
    for k in actionSubtreeDict.keys():
        rospy.logwarn("action name %s", k )
        seq.append(actionSubtreeDict[k][0])
    converted_tree.append(seq)

def reset_all():
    global actionSubtreeDict, converted_tree
    actionSubtreeDict = {}
    converted_tree = None

############################################################################################################


def create_bt(req):
    # print(req.pddl_plan)
    # raise NotImplementedError
    rospy.logwarn("got request ...")
    if req.type == str("new"):
        tree = createInitialTree(req.goal_condition)
    if req.type == str("update"):
        tree = updateTree(req.bt, req.goal_condition)
    if req.type == str("from_pddl"):
        tree = treeFromPddlPlan(req.pddl_plan)

    return tree

def convert_to_bt(req):
    rospy.logwarn("got request ...")
    if req.type == str("from_pddl"):
        tree = treeFromPddlPlan(req.pddl_plan)

    return tree

def create_bts_server():
    s = rospy.Service('/get_bt_service', GetBT, create_bt)
    print("Ready to generate behavior trees")
    rospy.spin()

def pddl_to_bt_server():
    s = rospy.Service('/pddl_to_bt', GetBT, convert_to_bt)
    print("Ready to generate behavior trees")
    rospy.spin()

def get_objects_info():
    global objectsInfo
    rospy.wait_for_service('/objects_info_service')
    try:
        srv = rospy.ServiceProxy('/objects_info_service', GetObjectsInfo)
        objects = []
        res = srv(str("get"), '','', False, objects)
        objectsInfo = res.objects
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.loginfo("got objects info")


def myhook():
    rospy.loginfo("nodes shutdown ")

if __name__ == '__main__':
    rospy.init_node('bts_handler',anonymous=False)
    t_bts = threading.Thread(name='bts_update', target=create_bts_server)
    t_pddl = threading.Thread(name='pddl_to_bts', target=pddl_to_bt_server)
    # t_plan_seder = threading.Thread(name='plan_sender', target=send_plan)
    t_bts.start()
    t_pddl.start()
    # t_plan_seder.start()
    rospy.on_shutdown(myhook)