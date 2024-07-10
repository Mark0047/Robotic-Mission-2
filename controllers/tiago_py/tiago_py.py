#
# The TiaGo robot is expected to approach four of the items in the environment.
# It must do so, avoid collisions with any of the static or mobile obstacles
# (i.e., walls, other robots and the cones), and it must reach the goals
# regardless of its starting position. The possible goal items are:
#  - the wooden box with the red top: `red`
#  - the wooden box with the green top: `green`
#  - the container with the ducks: `ducks`
#  - the container with the balls: `balls`
#
# The objects will be considered "close" when the distance to the TiaGo robot
# is lower than 0.8m.
#
# The input will be given using the keyboard, typing the previously mentioned
# shortened names (e.g., red, green, ducks, balls) in the 3D view (remember
# to click inside it) and pressing enter. Goals can be provided separated by
# commas. If new goals are provided before meeting the current goals, the new
# ones must be queued (the existing ones must be satisfied before continuing).
#
# The documentation is expected to be in the code, not an external document.
#

import math
import numpy as np
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range, get_goal_polygon
from pyperplan.planner import find_domain, HEURISTICS, search_plan, SEARCHES
import os
from shapely.geometry import Point, Polygon

robot = Robot()

timestep = int(robot.getBasicTimeStep())


l_motor = robot.getDevice("wheel_left_joint")
r_motor = robot.getDevice("wheel_right_joint")
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
l_motor.setPosition(math.inf)
r_motor.setPosition(math.inf)
l_motor.setVelocity(0)
r_motor.setVelocity(0)

keyboard = KeyboardReader(timestep)
goal =''


RESOLUTION=198


def call_planner(domain, problem):
    def get_callable_names(callables, omit_string):
        names = [c.__name__ for c in callables]
        names = [n.replace(omit_string, "").replace("_", " ") for n in names]
        return ", ".join(names)

    search_names = get_callable_names(SEARCHES.values(), "_search")
    heuristic = "hff"
    search="bfs"
    problem = os.path.abspath(problem)
    domain = os.path.abspath(domain)
    search = SEARCHES[search]
    heuristic = None
    use_preferred_ops = heuristic == "hffpo"
    solution = search_plan(
        domain,
        problem,
        search,
        heuristic,
        use_preferred_ops=use_preferred_ops,
    )
    return solution

def write_task(where, to):
    output = "task.pddl"
    fd = open(output, 'w')
    fd.write(f"""(define (problem go-to) (:domain example)
	(:objects
		bot - robot
		redroom greenroom ballsroom ducksroom - room
	)
	(:init
		(in bot {where}room)
		(cangowest greenroom redroom)
		(cangowest ballsroom ducksroom)

		(cangoeast redroom greenroom)
		(cangoeast ducksroom ballsroom)

		(cangonorth ducksroom redroom)
		(cangonorth ballsroom greenroom)

		(cangosouth greenroom ballsroom)
		(cangosouth redroom ducksroom)

	)
	(:goal
		(in bot {to}room)
	)
    )""")
    fd.close()


class ChangeRoom(object):
    # Intentionally poor way of working with state machines!
    # We expect better!
    def __init__(self, angle):
        self.target_angle = angle
        self.state = "orientate"

    def detect_doors(self, l, th=1):
        # detect opening
        open = None
        back_d = l[len(l)//2]
        for i in range(len(l)-1, 0, -1):
            d = l[i]
            if d-back_d < -th:
                open = i
            back_d = d
        print('open',open)
        # detect closing
        close = None
        back_d = l[len(l)//2]
        for i in range(0, len(l)):
            d = l[i]
            if d-back_d < -th:
                close = i
            back_d = d
        print('close',close)
        return ((open+close)/2-RESOLUTION//2)*math.pi/RESOLUTION, open, close

    def step(self, lidar_values, compass_values, gps_values=None):
        angle = math.atan2(compass_values[0], -compass_values[2])
        if self.state == "orientate":
            print('ORIENTATE', angle, self.target_angle)
            error = self.target_angle-angle
            print(error)
            if math.fabs(error)>0.15: # This is a constant control.
                if error < 0:        # You can try with a Propotional (or even PID)!
                    l_motor.setVelocity(1)
                    r_motor.setVelocity(-1)
                else:
                    l_motor.setVelocity(-1)
                    r_motor.setVelocity(+1)
            else:
                self.state = "approach"
                l_motor.setVelocity(0)
                r_motor.setVelocity(0)
        elif self.state == "approach":
            print('APPROACH')
            target_angle, o, c = self.detect_doors(lidar_values)
            print('APPROACH', o, c)
            l_motor.setVelocity(5+target_angle*2)
            r_motor.setVelocity(5-target_angle*2)
           
            next = False
            if o is not None:
                # print('o',o)
                if o < RESOLUTION//6:
                    next = True
            if c is not None:
                # print('c',c)
                if c>RESOLUTION-RESOLUTION//6:
                    next = True
            if next:
                self.state = "cross" 
                self.cross_steps = 1200
        elif self.state == "cross":
            print("CROSS")
            self.cross_steps -=1 
            # print('self.cross_steps',self.cross_steps)
            l_motor.setVelocity(0.8)
            r_motor.setVelocity(0.8)
            H = RESOLUTION//2
            if np.mean(np.array(lidar_values[H-10:H+10]))< 1.4 or self.cross_steps == 0:
                self.state = "done"
                print("DONE")
        elif self.state == "done":
            l_motor.setVelocity(0.3)
            r_motor.setVelocity(-0.3)
            

class Chill(object):
    
    def __init__(self, target_polygon):
        print('chilling')
        self.tick = 0
        self.state = "approach_object"
        self.object_detected = False
        self.object_position = None
        self.target_polygon = target_polygon
        print

    def approach_object(self, compass_values, gps_values):
        # Approach the object based on lidar position
        
        robot_position = Point(gps_values[0], gps_values[1])
        print(self.target_polygon)
        centroid = self.target_polygon.centroid
        target_x, target_y = centroid.x, centroid.y
        current_x, current_y = robot_position.x, robot_position.y
        if self.target_polygon.distance(Point(gps_values[0], gps_values[1])) < 0.8:
            self.state = ''
            # print('reached goal')
            return
        print('current_x, current_y, target_x, target_y',current_x, current_y, target_x, target_y)
        target_angle = math.atan2(target_y - current_y, target_x - current_x)
        # print('compass_values2',compass_values)
        current_angle = math.atan2(compass_values[0], -compass_values[2])
        # print('compass_values3',compass_values)
        heading_error = target_angle - current_angle
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Calculate direction vector
        # Control signals
        heading_control = 2.0 * heading_error  # Proportional control constant
        forward_velocity = 3.0  # Constant forward speed

        # Set velocities
        l_motor.setVelocity(forward_velocity - heading_control)
        r_motor.setVelocity(forward_velocity + heading_control)
    
    def adjust_heading(current_angle, target_angle):
        error = target_angle - current_angle
        if error > math.pi:
            error -= 2 * math.pi
        elif error < -math.pi:
            error += 2 * math.pi
        return error


    def step(self, lidar_values, compass_values, gps_values=None):
        if self.state == "approach_object":
            print('approaching objects')
            self.approach_object(compass_values, gps_values)
        else:
            print('Not approaching objects')
            l_motor.setVelocity(math.cos(0.01*self.tick)*1.5)
            r_motor.setVelocity(-math.cos(0.01*self.tick)*1.5)
            self.tick += 1 
            
def get_room(values):
    print('get_roomvalues',values)
    if values[0]>=0:
        if values[1]>=0:
            return "red"
        else:
            return "green"
    else:
        if values[1]>=0:
            return "ducks"
        else:
            return "balls"
        


behaviour = None
current_action = None
target_polygon = None

while (robot.step(timestep) != -1):
    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')
        goal = command
        if goal is not None and goal != '':
            target_polygon = get_goal_polygon(command)
            print('The robot is next to', get_goals_in_range(*gps.getValues()[0:2], goal))

    room = get_room(gps.getValues())
    write_task(room, goal)
    # print(room)

    solution = call_planner("domain.pddl", "task.pddl")
    if solution is not None:
        # for s in solution:
        #     print(s)
        if len(solution)>0:
            first_action = solution[0].name[1:].split()[0]
        else:
            first_action = "chill"
        print(f'{first_action=}')

        allow_action_change = True
        if behaviour is not None:
            if behaviour.state == "cross":
                allow_action_change = False

        if allow_action_change and first_action != current_action:
            current_action = first_action
            if first_action == "movewest":
                behaviour = ChangeRoom(math.pi/2) # 90
            elif first_action == "movenorth":
                behaviour = ChangeRoom(0) # 0
            elif first_action == "moveeast":
                behaviour = ChangeRoom(-math.pi/2)
            elif first_action == "movesouth":
                behaviour = ChangeRoom(math.pi)
            elif first_action == "chill":
                behaviour = Chill(target_polygon)
            else:
                print("to do")
            
        lidar_values = lidar.getRangeImage()[1:-1]
        compass_values = compass.getValues()
        behaviour.step(lidar_values, compass_values, gps.getValues()[0:2])

# while (robot.step(timestep) != -1):
#     command = keyboard.get_command()
#     if command is not None:
#         print(f'Got command: {command}')

#     print('The robot is next to', get_goals_in_range(*gps.getValues()[0:2]))
    


