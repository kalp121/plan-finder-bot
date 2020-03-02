#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
import problem
import heapq
import argparse
import os
import json
from std_msgs.msg import String

from planning.srv import MoveActionMsg
from planning.srv import PlaceActionMsg
from planning.srv import PickActionMsg
from problem import Helper

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

###########################################
## Set Domain and Problem file path here ##
###########################################
DOMAIN_FILE_PATH = ROOT_PATH + "/domain.pddl"
PROBLEM_FILE_PATH = ROOT_PATH + "/problem.pddl"

#############################
## Set Planners' Path Here ##
#############################
FD_PATH = ROOT_PATH + "/planners/FD/fast-downward.py"
FF_PATH = ROOT_PATH + "/planners/FF/ff"

##################################
## Set FD's plan file path here ##
##################################
FD_PLAN_FILE = ROOT_PATH + "/sas_plan"

#######################################################
## Change default parameters to FD or Pyperplan here ##
#######################################################
FD_DEFAULT_PARAMS = "--search \"lazy_greedy([ff()], preferred=[ff()])\""

parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument('-step', help="Step to execute:\n1. Generate plan\n2. Perform downward refinement and execute",
                    metavar='1', action='store', dest='step', default="1", type=int)
parser.add_argument("-planner", metavar="FD", dest='planner', default='FD', help="which planner to use {FF, FD}")
PP_DEFAULT_PARAMS = "-H hff -s gbf"
parser.add_argument("-plan_file", metavar="", dest='plan_file', default="", help="location of plan file")
parser.add_argument("-params", metavar="", dest='params', default="", help="parameters to the planner")

"""
Do not change anything above this line, except if you want to import some package.

"""


class Refine:
    """
    This class has code to refine the high level actions used by PDDL to the the low level actions used by the TurtleBot.

    """

    def __init__(self, plan_file, planner):
        """
        :param plan_file: Path to the file where plan is stored. This is generally stored at the path from where the planner was called.
        :type plan_file: str
        :param planner: Planner that was used to generate this plan. This is needed to parse the plan and convert to/store in a data structure in Python. Only possible values are FF and FD.
        :type planner: str

        Attributes:
            **status_subscriber**: To subscribe to the "/status" topic, which gives the State of the robot.

            **actions_queue** (tuple(action_name, action_params)): Used to store the refined action tuples. You can decide what parameters you want to store for each action. Store the parameters that you need to pass to execute_<action_name>_action methods of RobotActionServer class.

            **action_index** (int): Stores the index of action that needs to be sent to RobotActionServer for execution.
        """

        rospy.init_node('listener', anonymous=True)
        self.status_subscriber = rospy.Subscriber('/status', String, self.status_callback)
        self.helper = problem.Helper()
        self.action_index = 0
        self.actions_queue = self.refine_plan(plan_file, planner)
        self.execute_action()
        rospy.spin()

    def status_callback(self, data):
        """
        Callback function for subscriber to the `/status` rostopic

        """
        if (data.data == "Idle"):
            self.execute_action()

    def execute_action(self):
        """
        This method picks an action from self.actions and send it to RobotActionServer for execution.

        :rtype: None

        """
        h = Helper()
        if (self.action_index >= len(self.actions_queue)):
            return
        action = self.actions_queue[self.action_index]
        self.action_index += 1
        print("Executing ", action)

        if isinstance(action, tuple):

            if len(action) == 2:
                h.execute_pick_action(action[0], action[1])

            else:
                h.execute_place_action(action[0], action[1], action[2])

        else:
            h.execute_move_action(action)

        # Your code here

    def refine_plan(self, plan_file, planner):
        """
        Perform downward refinement to convert the high level plan into a low level executable plan.

        :param plan_file: Path to the file where plan is stored. This is generally stored at the path from where the planner was called.
        :type plan_file: str
        :param planner: Planner that was used to generate this plan. This is needed to parse the plan and convert to/store in a data structure in Python. Only possible values are FF and FD.
        :type planner: str

        :returns: List of refined action tuples that will be added to the execution queue.
        :rtype: list(tuples)

        .. note::

            .. hlist::
                :columns: 1

                * Parse the plan from plan_file. This has to be according to the planner you are using.
                * use get_path(current_state, load_locations) to refine Move action.
                * Subject of the book and bin does not match.
                * Robot Location is not within the load location of the bin, i.e. robot is not in the viscinity of the bin.


        """
        # / home / kalp / catkin_ws / src / planning / scripts / refinement.py

        h = Helper()
        init_state = h.get_initial_state()
        action_list = []
        success = True
        dirname=os.path.dirname(__file__)
        filename=os.path.join(dirname,"../objects.json")
        with open(filename) as json_file:
            try:
                objects = json.load(json_file, parse_float=float)
            except (ValueError, KeyError, TypeError):
                print "JSON error"
        plan = []

        with open(plan_file, "r") as f:
            try:
                plan = f.readlines()
            except (ValueError, KeyError, TypeError):
                print ("File error")
        plan = [x.strip() for x in plan]
        plan = [x.strip("(") for x in plan]
        plan = [x.strip(")") for x in plan]
        plan = [x.split() for x in plan]

        start_loc = init_state
        prev_loc = init_state
        for item in plan:
            if item[0] == "move":

                if item[3] == "tbot3_init_loc":

                    start_loc = prev_loc
                    goal_loc = [init_state.X, init_state.Y]



                else:

                    # obj1=item[2]
                    # obj1_name=obj1[:-5]
                    # obj1_type=obj1_name.split('_')[0]
                    obj2 = item[3]
                    obj2_name = obj2[:-5]
                    obj2_type = obj2_name.split('_')[0]
                    start_loc = prev_loc

                    if obj2_type == 'book':
                        goal_loc = objects['books'][obj2_name]['load_loc']
                    else:
                        goal_loc = objects['bins'][obj2_name]['load_loc']

                actions, final_loc, goal = self.get_path(start_loc, goal_loc)
                prev_loc = final_loc
                if goal:
                    action_list.append(actions)
                else:
                    success = False


            elif item[0] == "pick":
                book_name = item[1]
                action_list.append((book_name, prev_loc))

            elif item[0] == "place":
                book_name = item[1]
                bin_name = item[3]
                action_list.append((book_name, bin_name, prev_loc))

        if not success: action_list = []
        action_list = [x for x in action_list if x != []]
        #print(action_list)

        return action_list

    # --------------- HELPER FUNCTIONS --------------- #

    def is_goal_state(self, current_state, goal_state):
        """
        Checks if the current_state is goal_state or not.
        If you are wondering why we are checking orientation, remember this is a different Homework. :)

        """
        if (
                current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation):
            return True
        return False

    def get_manhattan_distance(self, from_state, to_state):
        """
        Returns the manhattan distance between 2 states

        """
        return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

    def build_goal_states(self, locations):
        """
        Creates a State representations for given list of locations

        """
        states = []
        for location in locations:
            states.append(problem.State(location[0], location[1], "EAST"))
            states.append(problem.State(location[0], location[1], "WEST"))
            states.append(problem.State(location[0], location[1], "NORTH"))
            states.append(problem.State(location[0], location[1], "SOUTH"))
        return states

    def get_path(self, init_state, goal_locations):
        """
        This method searches for a path from init_state to one of the possible goal_locations

        :param init_state: Current state of robot
        :type init_state: State
        :param goal_locations: list of target locations to search the path e.g. [(x1, y1), (x2, y2), ..]. This is important if there are multiple books of a subject.
        :type goal_locations: list(State)

        :returns:
            .. hlist::
                :columns: 1

                * **action_list**: list of actions to execute to go from source to target
                * **final_state**: target state that is reached (will be one of goal_locations)
                * **goal_reached**: True/False indicating if one of the goal_locations was reached

        """
        final_state = None
        goal_states = self.build_goal_states(goal_locations)
        goal_reached = False
        for goal_state in goal_states:  # search for any of the load locations
            possible_actions = self.helper.get_actions()
            action_list = []

            state_queue = []
            heapq.heappush(state_queue, (self.get_manhattan_distance(init_state, goal_state), 0, (init_state, [])))
            visited = []
            state_cost = {}
            insert_order = 0

            while len(state_queue) > 0:
                top_item = heapq.heappop(state_queue)
                current_cost = top_item[0]
                current_state = top_item[2][0]
                current_actions = top_item[2][1]

                if (current_state in visited):
                    continue

                if (self.is_goal_state(current_state, goal_state)):
                    goal_reached = True
                    break

                visited.append(current_state)
                for action in possible_actions:
                    nextstate, cost = self.helper.get_successor(current_state, action)
                    cost = self.get_manhattan_distance(nextstate, goal_state)  # manhattan distance heuristc
                    key = (nextstate.x, nextstate.y, nextstate.orientation)
                    if (nextstate.x == -1 and nextstate.y == -1):
                        continue
                    # if a new state is found then add to queue
                    if (nextstate not in visited and key not in state_cost.keys()):
                        heapq.heappush(state_queue, (cost, insert_order, (nextstate, current_actions + [action])))
                        insert_order += 1
                        state_cost[key] = cost

            if (self.is_goal_state(current_state, goal_state)):
                action_list = current_actions
                final_state = current_state
                goal_reached = True
                break

        return action_list, final_state, goal_reached


# ------------------------------------------------ #
# DO NOT MODIFY THE CODE BELOW

def generate_plan(planner, params):
    '''
    Run a planner to evaluate problem.pddl and domain.pddl to generate a plan.
    Writes the plan to an output file
    '''
    command = ""
    if planner == "FF":
        command += FF_PATH + " -o "
    elif planner == "FD":
        command += FD_PATH + " "

    command += DOMAIN_FILE_PATH + " "

    if planner == "FF":
        command += " -f "

    command += PROBLEM_FILE_PATH + " "

    if planner == "FD" and params == "":
        command += FD_DEFAULT_PARAMS
    else:
        command += params

    print(command)
    os.system(command)


if __name__ == "__main__":

    args = parser.parse_args()

    if args.planner == None or args.planner == "":
        print "Planner not provided, using FD by default"
    elif args.planner not in ["FD", "FF"]:
        print "Incorrect Planner provided. Aborting"
        exit(1)
    else:
        print "Using " + args.planner + " Planner"

    plan_file = args.plan_file
    if plan_file == None or plan_file == "":
        plan_file = PROBLEM_FILE_PATH + ".soln"
        if args.planner == "FD":
            plan_file = FD_PLAN_FILE
        print "plan_file not provided. Setting plan_file = " + plan_file

    if args.step == None:
        print "Step missing"
        os.system(ROOT_PATH + "scripts/refinement.py -h")
        exit(1)

    if args.params == None or args.params == "":
        print "Parameters to planner not provided. Using default parameters"

    if args.step == 1:
        generate_plan(args.planner, args.params)
    elif args.step == 2:
        try:
            Refine(plan_file, args.planner)
        except rospy.ROSInterruptException:
            pass
