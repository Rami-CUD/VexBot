PRE COMP MODIFFICATIONS:
#region VEXcode Generated Robot Configuration
from vex import *
# Brain should be defined by default
brain=Brain()

# Robot configuration code
Motor_R1 = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
Motor_L1 = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
Left_Sensor = Line(brain.three_wire_port.b)
Right_Sensor = Line(brain.three_wire_port.a)
Motor_R2 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
Motor_L2 = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
# vex-vision-config:begin
vision_13__PULA = Signature(1, 5007, 5931, 5468,-515, 97, -208,4, 0)
# vision_13__BUGHAW = Signature(2, -2145, -1423, -1784,3071, 4633, 3852,1.2, 0)
# vision_13__BUGHAW = Signature(2, -1713, -865, -1290,2089, 3511, 2800,1.2, 0)
vision_13__BUGHAW = Signature(2, -1903, -1113, -1508,2561, 4097, 3330,1.9, 0)
# vision_13__CLOSEBLUE = Signature(2, -1713, -865, -1290,2089, 3511, 2800,1.2, 0)
vision_13 = Vision(Ports.PORT13, 110, vision_13__PULA, vision_13__BUGHAW)
# vision_13 = Vision(Ports.PORT13, 85, vision_13__PULA, vision_13__BUGHAW)
# close_vision_13 = Vision(Ports.PORT13, 143, vision_13__CLOSEBLUE)
# vex-vision-config:end
Claw_Motor = Motor(Ports.PORT1, GearSetting.RATIO_36_1, False)

Middle_Sensor = Line(brain.three_wire_port.c)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#region VEXcode Generated Robot Configuration
#endregion VEXcode Generated Robot Configuration
class Claw_Controller:
    SPIN_DEGREES = 90
    def __init__(self, claw_motor: Motor, is_open=True):
        self.claw_object = claw_motor
        self.is_open = is_open
    
    def toggle(self):
        if self.is_open:
            self.close()
        else:
            self.open()
        self.is_open = not self.is_open

    def open(self, *,forced=False):
        if not self.is_open or forced:
            self.claw_object.spin_for(REVERSE, Claw_Controller.SPIN_DEGREES, DEGREES)
            self.is_open = True

    def close(self, *, forced=False):
        if self.is_open or forced:
            self.claw_object.spin_for(FORWARD,Claw_Controller.SPIN_DEGREES, DEGREES)
            self.is_open = False

class Directions():
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3
    RIGHT = 1
    LEFT = 0

    
    # Technically both turn methods could be combined into one, but I seperated them taking
    # into account that the robot itself might have them seperated

# def turn_right(current_direction: int, *, times=1):
#     Motor_L1.spin_to_position(180, DEGREES)
#     Motor_L2.spin_to_position(180, DEGREES)
#     Motor_R1.spin_to_position(-180, DEGREES)
#     Motor_R2.spin_to_position((-180,  DEGREES))
#     return (current_direction + abs(times)) % 4
# def turn_left(current_direction: int, *, times=1):
#     Motor_L1.spin_to_position(180, DEGREES)
#     Motor_L2.spin_to_position(180, DEGREES)
#     Motor_R1.spin_to_position(-180, DEGREES)
#     Motor_R2.spin_to_position((-180, DEGREES))
#     return (current_direction - abs(times)) % 4

# main class NavSys manages mapping and the 4 cardinal directions (north, east, south, west)
class NavSys:
    RED_GOAL = (5, 2)
    BLUE_GOAL = (0, 2)

    # constructor class of class NavSys initializes the map and the initial direction as North
    def __init__(self, size):
        self.current_direction = Directions.EAST
        self.current_coordinate = [1,1]
        self.claw = Claw_Controller(Claw_Motor, is_open=True)
        self.mapping_mode = True
        self.cone_coords = {(1, 4)}
        # self.blue_queue = []
        # self._initialize_blue()
        self.take_snapshot()

    # method print_map has no other use aside from easier visualization in the terminal
    def take_snapshot(self, min_red_width=24, min_blue_width=22):
        self.red_objects = vision_13.take_snapshot(vision_13__PULA)
        self.blue_objects = vision_13.take_snapshot(vision_13__BUGHAW)
        # self.blue_queue.pop(0)
        # if self.blue_objects and self.blue_objects[0].height >= 30:
        #     self.blue_queue.append(self.blue_objects[0].width)
        # else:
        #     self.blue_queue.append(0)

        self.blue_object_exists = (self.blue_objects[0].width > min_blue_width and self.blue_objects[0].height >= 30) if self.blue_objects else None
        self.red_object_exists = (self.red_objects[0].width > min_red_width and self.red_objects[0].height >= 30) if self.red_objects else None
        # if self.blue_object_exists:
        #     brain.screen.print(self.blue_objects[0].width)
        #     brain.screen.new_line()
        #     print(self.blue_objects[0].width)
        # else:
        #     print(0)
        # if self.red_object_exists:
        #     brain.screen.print(self.red_objects[0].width)
        #     brain.screen.new_line()
    
    # def _initialize_blue(self):
    #     for _ in range(5):
    #         blue_object = vision_13.take_snapshot(vision_13__BUGHAW)
    #         if blue_object:
    #             self.blue_queue.append(blue_object[0].width)
    #         else:
    #             self.blue_queue.append(0)

    
    def _avg(self, lst:list):
        return sum(lst)/len(lst)





    def mov_straight(self, value=4, unit=VOLT):
        Motor_R1.spin(FORWARD, value, unit)
        Motor_R2.spin(FORWARD, value, unit)
        Motor_L1.spin(FORWARD, value, unit)
        Motor_L2.spin(FORWARD, value, unit) 

    def mov_backwards(self):
        Motor_R1.spin(REVERSE, 4, VOLT)
        Motor_R2.spin(REVERSE, 4, VOLT)
        Motor_L1.spin(REVERSE, 4, VOLT)
        Motor_L2.spin(REVERSE, 4, VOLT)

    def left(self, value=3, unit=VOLT):
        Motor_R1.spin(FORWARD, value, unit)
        Motor_R2.spin(FORWARD, value, unit)
        Motor_L1.spin(REVERSE, value, unit)
        Motor_L2.spin(REVERSE, value, unit)

    def right(self,value=3, unit=VOLT):
        Motor_R1.spin(REVERSE, value, unit)
        Motor_R2.spin(REVERSE, value, unit)
        Motor_L1.spin(FORWARD, value, unit)
        Motor_L2.spin(FORWARD, value, unit)

    def turn_90(self, change: int):
        if change == 1:
            right_motor_dir = REVERSE
            left_motor_dir = FORWARD
        if change == 0:
            right_motor_dir = FORWARD
            left_motor_dir = REVERSE
        Motor_R1.spin(right_motor_dir)
        Motor_R2.spin(right_motor_dir)
        Motor_L1.spin(left_motor_dir)
        Motor_L2.spin(left_motor_dir)
        # wait(940, MSEC)
        for sec in range(940):
            if (sec > 500) and (Middle_Sensor.reflectivity() < 75):
                break
            wait(1, MSEC)
            
        self.stop()
        wait(100, MSEC)

    def turn_180(self):
        Motor_R1.spin(FORWARD)
        Motor_R2.spin(FORWARD)
        Motor_L1.spin(REVERSE)
        Motor_L2.spin(REVERSE)
        wait(1800, MSEC)
        self.stop()


    def stop(self):
        Motor_R1.stop()
        Motor_R2.stop()
        Motor_L1.stop()
        Motor_L2.stop()
        

    def on_cross(self) -> bool:
            reflection_threshold = 75
            left, middle, right = Left_Sensor.reflectivity(), Middle_Sensor.reflectivity(), Right_Sensor.reflectivity()
            middle_detected = middle < reflection_threshold
            both_sides_detected = (left < reflection_threshold) and (right < reflection_threshold) 
            return middle_detected and both_sides_detected
    
    def any_object_exists(self):
        return self.red_object_exists or self.blue_object_exists
    
    def straight(self, direction, speed=4, extra_move = True, update_position= True):
        # timer = 0
        # def snapshot_within_timer(min_red_width=24, min_blue_width=20):
        #     nonlocal timer
        #     if timer >= 200:
        #         self.take_snapshot(min_red_width, min_blue_width)
        #     else:
        #         timer += 1

        if update_position:
            snapshot = lambda: self.take_snapshot() 
        else:
            snapshot = lambda: self.take_snapshot(min_red_width=57, min_blue_width=53)
        snapshot()
        while not self.on_cross() and (not self.any_object_exists() or not self.claw.is_open):
            snapshot()
            self.mov_straight(value=speed)
            while Left_Sensor.reflectivity() < 60 and not Middle_Sensor.reflectivity() < 30 and not self.on_cross() and (not self.any_object_exists() or not self.claw.is_open):
                snapshot()
                self.left(60, RPM) # type: ignore
            while Right_Sensor.reflectivity() < 60 and not Middle_Sensor.reflectivity() < 30 and not self.on_cross() and (not self.any_object_exists() or not self.claw.is_open):
                snapshot()
                self.right(60, RPM) # type: ignore
        if not self.any_object_exists and extra_move:
            self.mov_straight()
            wait(150, MSEC)
        elif extra_move:
            self.mov_straight()
            wait(100, MSEC)
        self.stop()
        if not update_position:
            return
        plane = 0 if direction in (Directions.EAST, Directions.WEST) else 1
        displacement = 1 if direction in (Directions.NORTH, Directions.EAST) else -1
        self.current_coordinate[plane] += displacement


    # method faceto checks what direction the robot needs to face and changes the robot's direction
    def faceto(self, target_direction, ignore_line=False):
        difference = target_direction - self.current_direction # type: ignore
        NOT_THREE = abs(difference) != 3
        # The behavior when the difference is three is the opposite of normal behavior :/
        positive_difference = difference > 0 if NOT_THREE else not(difference > 0)
        turn_direction = Directions.RIGHT if positive_difference > 0 else Directions.LEFT
        # Num of rotations is the same as the difference unless it's three in which case we only need one rotation
        rotations = abs(difference) if NOT_THREE else 1
        if rotations == 0:
            ignore_line = True
        for rotation in range(rotations):
            if rotation == 1:
                self.mov_backwards()
                wait(99, MSEC)
                self.stop()
            self.turn_90(turn_direction)
        self.current_direction = target_direction
        if ignore_line:
            return
        if turn_direction == Directions.RIGHT:
            overshoot_condition = (Middle_Sensor.reflectivity() < 75 or Left_Sensor.reflectivity() < 75)  
        else:
            overshoot_condition = (Middle_Sensor.reflectivity() < 75 or Right_Sensor.reflectivity() < 75)  

        if overshoot_condition:
            return
        while not (Middle_Sensor.reflectivity() < 75):
            if turn_direction == Directions.RIGHT:
                self.right(40, RPM) # type: ignore
            else:
                self.left(40, RPM) # type: ignore
        self.stop()
        
    def goto(self, target_coordinate, reversed=False,):
        x_change = target_coordinate[0] - self.current_coordinate[0]
        y_change = target_coordinate[1] - self.current_coordinate[1]
        WAIT_INTERVAL = 500
        displacements = (x_change, y_change) if not reversed else (y_change, x_change)
        x_index = 0 if not reversed else 1
        for i, displacement in zip(range(2), displacements):
            if displacement == 0:
                continue
            if i == x_index:
                target_direction = Directions.EAST if displacement > 0 else Directions.WEST
            else:
                target_direction = Directions.NORTH if displacement > 0 else Directions.SOUTH
            self.faceto(target_direction) # type: ignore
            for steps in range(abs(displacement)):
                self.straight(target_direction)
                if self.any_object_exists():
                    # brain.screen.new_line()
                    # brain.screen.print(self.blue_object_exists, self.red_object_exists)

                    self.take_snapshot()
                    self._handle_object_detection(target_coordinate, reversed)
                    return True
                # wait(WAIT_INTERVAL, MSEC)

    def _handle_object_detection(self, target_coordinate, reversed):
        # brain.screen.print("detected")
        # brain.screen.new_line()
        # self.claw.close()
        # self.goto(target_coordinate, reversed)
        pass
    
    def find_nearest_cone(self):
        def distance(source, target):
            x_steps = abs(target[0] - source[0])
            y_steps = abs(target[1] - source[1])
            return x_steps + y_steps
        
        nearest_cone_coord = []
        minimum_distance = 0
        for coord in self.cone_coords:
            current_distance = distance(self.current_coordinate, coord)
            if len(nearest_cone_coord) == 0 or current_distance < minimum_distance:
                nearest_cone_coord = coord
                minimum_distance = current_distance
        if not nearest_cone_coord:
            raise RuntimeError("No cones")
        return nearest_cone_coord

    def dijkstra(self, start, target, obstacles:set, grid_size=6):
        # Define the directions for movement: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        obstacles = obstacles.union({(x, y) for x in range(grid_size) for y in range(grid_size) if x == 5 or y == 5 or x == 0 or y == 0})
        obstacles = obstacles.difference({target})

        # Initialize distances dictionary with infinity for all nodes except the start node
        distances = { (x, y): float('inf') for x in range(grid_size) for y in range(grid_size) }
        distances[start] = 0
        # Initialize the dictionary to store the shortest path
        shortest_path = {start: [start]}
        # Initialize the list to act as our priority queue
        pq = [start]

        while pq:
            # Find the node with the smallest distance
            current_node = min(pq, key=lambda node: distances[node])
            pq.remove(current_node)
            current_distance = distances[current_node]
            current_x, current_y = current_node

            # If this node is the target, we can return the result early
            if current_node == target:
                return distances[target], shortest_path[target]

            # Explore neighbors
            for direction in directions:
                neighbor_x, neighbor_y = current_x + direction[0], current_y + direction[1]
                neighbor = (neighbor_x, neighbor_y)

                # Check if the neighbor is within the grid bounds and not an obstacle
                if 0 <= neighbor_x < grid_size and 0 <= neighbor_y < grid_size and neighbor not in obstacles:
                    distance = current_distance + 1  # Each move has a weight of 1

                    # Only consider this new path if it's better
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        if neighbor not in pq:
                            pq.append(neighbor)
                        # Update the shortest path
                        shortest_path[neighbor] = shortest_path[current_node] + [neighbor]

        return float('inf'), []  # Return infinity and empty path if the target is unreachable
    
    def center(self):
        while not Left_Sensor.reflectivity() < 60:
            self.right(40, RPM) # type: ignore
        while Left_Sensor.reflectivity() < 60 and not Middle_Sensor.reflectivity() < 90 :
            self.left(42, RPM) # type: ignore
        wait(110, MSEC)
        while Right_Sensor.reflectivity() < 60 and not Middle_Sensor.reflectivity() < 75:
            self.right(40, RPM) # type: ignore
        self.stop()
    
    def goto_and_grab_nearest_cone(self):
        nearest_cone = self.find_nearest_cone()
        self.goto(nearest_cone)
        self.mapping_mode = False
        # self.center()
        self.straight(self.current_direction, speed=2, extra_move=False, update_position=False)
        brain.screen.new_line()
        brain.screen.print(self.blue_object_exists, self.red_object_exists)
        self.claw.close()
        if self.red_object_exists:
            goal_direction = Directions.EAST
            goal_coords = NavSys.RED_GOAL
        elif self.blue_object_exists:
            goal_direction = Directions.WEST
            goal_coords = NavSys.BLUE_GOAL
        else:
            self.stop()
            self.claw.open()
            # raise RuntimeError("Not holding a can")
            return 
        self.claw.is_open = False
        self.straight(self.current_direction, update_position=False)
        self.mov_straight()
        wait(100, MSEC)
        distance, path = self.dijkstra(tuple(self.current_coordinate), goal_coords, self.cone_coords)
        brain.screen.new_line()
        brain.screen.print(path)
        self.mapping_mode = True
        for route_coord in path:
            self.goto(route_coord)
        self.faceto(goal_direction)
        self.drop_cone()
        self.cone_coords.remove(nearest_cone) # type: ignore

    def drop_cone(self):
        self.mov_straight()
        wait(350, MSEC)
        self.stop()
        self.claw.open()
        self.mov_backwards()
        wait(500, MSEC)
        self.stop()

        

    def operate(self):
        # obj = []
        # obj_2 = []

        # for _ in range(1000):
        #     object = vision_13.take_snapshot(vision_13__CLOSEBLUE)
        #     object_2 = vision_13.take_snapshot(vision_13__BUGHAW)
        #     if object:
        #         obj.append(object_2[0].width)
        #     if object_2:
        #         obj_2.append(object_2[0].width)
        # print((sum(obj))/len(obj))
        # print((sum(obj_2))/len(obj_2))
        # self.claw.claw_object.spin_for(REVERSE, 35, DEGREES)
        self.scan_grid()
        while self.cone_coords:
            self.goto_and_grab_nearest_cone()
        # self.goto_and_grab_nearest_cone()
        # -16.2
        # self.claw.open(forced=True)
        # _, x = self.dijkstra((0,3), (3,1), {(0,3)})
        # self.center()
        # self.straight(self.current_direction)
        # while True:
        #     self.take_snapshot()
        pass




    def scan_grid(self, grid_size=6):
        visited = set()  # Initialize visited array
        obstacles = {(x, y) for x in range(6) for y in range(6) if x == 5 or y == 5 or x == 0 or y == 0}  # Initialize cones array
        for x in range(1, grid_size - 1):
            if x % 2 == 1:  # Odd column (1, 3, 5), traverse top to bottom
                for y in range(1, grid_size - 1):
                    if (x,y) in visited or (x,y) in obstacles:
                        continue
                    while True:
                        retry = False
                        _, path = self.dijkstra(tuple(self.current_coordinate), (x, y), obstacles)
                        for coord in path:
                            has_cone = robot.goto(coord)
                            if has_cone:
                                robot.faceto((robot.current_direction + 2) % 4) # type: ignore
                                robot.take_snapshot()
                                robot.straight(robot.current_direction) # type: ignore
                                obstacles.add(coord)
                                self.cone_coords.add(coord)
                                if coord != (x, y):
                                    retry = True
                            else:
                                visited.add(coord)
                            if retry:
                                break
                        if not retry:
                            break

                            

            else:  # Even column (2, 4), traverse bottom to top
                for y in range(grid_size - 2, 0, -1):
                    if (x,y) in visited or (x,y) in obstacles:
                        continue
                    while True:
                        retry = False
                        _, path = self.dijkstra(tuple(self.current_coordinate), (x, y), obstacles)
                        for coord in path:
                            has_cone = robot.goto(coord)
                            if has_cone:
                                robot.faceto((robot.current_direction + 2) % 4) # type: ignore
                                robot.take_snapshot()
                                robot.straight(robot.current_direction) # type: ignore
                                obstacles.add(coord)
                                self.cone_coords.add(coord)
                                if coord != (x, y):
                                    retry = True
                            else:
                                visited.add(coord)
                            if retry:
                                break
                        if not retry:
                            break
        brain.screen.print(self.cone_coords)
                            

    # def scan_grid(self, grid_size=6):
    #     visited = [[False] * grid_size for _ in range(grid_size)]  # Initialize visited array
    #     cones = [[False] * grid_size for _ in range(grid_size)]  # Initialize cones array
    #     for x in range(6):
    #         for y in range(6): 
    #             if x == 5 or y == 5 or x == 0 or y == 0:
    #                 cones[x][y] = True
    #     def is_valid_move(x, y):
    #         # Check if the move is within the grid boundaries, not visited before, and does not have a cone
    #         return 0 <= x < grid_size and 0 <= y < grid_size and not visited[x][y] and not cones[x][y]

    #     def dfs(use_y_counter=0):
    #         if use_y_counter > 0:
    #             reversed = True
    #             use_y_counter -= 1
    #         else:
    #             reversed = False
    #         x,y = self.current_coordinate
    #         # Define possible moves: up, down, left, right
    #         moves = [(0, 1), (1, 0), (-1, 0), (0, -1)]

    #         # Mark the current node as visited
    #         visited[x][y] = True

    #         # Explore all possible moves from the current node
    #         for move in moves:
    #             new_x, new_y = x + move[0], y + move[1]
    #             if is_valid_move(new_x, new_y):
    #                 has_cone = robot.goto([new_x, new_y], reversed)
    #                 if has_cone:
    #                     if move[0] != 0:
    #                         use_y_counter += 1
    #                     robot.faceto((robot.current_direction + 2) % 4) # type: ignore
    #                     robot.take_snapshot()
    #                     robot.straight(robot.current_direction) # type: ignore
    #                     cones[new_x][new_y] = True
    #                     self.cone_coords.add((new_x,new_y))
    #                     continue
    #                 dfs(use_y_counter)
    #     dfs()



robot = NavSys(5)
def go_through_entire_map():
    y_axis = 5
    for i in range(5):
        robot.goto([i,y_axis])
        if i + 1 < 6:
            robot.goto([i+1,y_axis])
        y_axis = 5 - y_axis




robot.operate()
# robot.scan_grid()
