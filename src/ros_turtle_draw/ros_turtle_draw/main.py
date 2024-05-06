import time
from math import pi

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from turtlesim.srv import Kill, SetPen, Spawn

zero_degrees = 0.0
ninety_degrees = pi / 2
one_eighty_degrees = pi
two_seventy_degrees = (3 * pi) / 2

class Instruction:
    def __init__(self, x: float, z: float):
        self.x = x
        self.z = z

    def export(self):
        return Twist(linear=Vector3(x=self.x), angular=Vector3(z=self.z))


class Position:
    def __init__(self, x: float, y: float, rotation: float = 0.0):
        if rotation not in [zero_degrees, ninety_degrees, one_eighty_degrees, two_seventy_degrees, -zero_degrees, -ninety_degrees, -one_eighty_degrees, -two_seventy_degrees]:
            raise Exception("Invalid rotation value. Must be 0, pi/2, pi, or 3pi/2")

        self.x = x
        self.y = y
        self.rotation = rotation

    def add_rotation(self, rotation: float):
        if rotation not in [zero_degrees, ninety_degrees, one_eighty_degrees, two_seventy_degrees, -zero_degrees, -ninety_degrees, -one_eighty_degrees, -two_seventy_degrees]:
            raise Exception("Invalid rotation value. Must be 0, pi/2, pi, or 3pi/2")

        # loop around 360 degrees
        self.rotation = (self.rotation + rotation) % (2 * one_eighty_degrees)


class TurtleDraw(Node):
    def __init__(self):
        super().__init__("ros_turtle_draw_gustavo")
        self.turtle_name = "turtle_gustavo"
        self.position = Position(0.0, 0.0)

        self.publisher_ = self.create_publisher(Twist, f"{self.turtle_name}/cmd_vel", 10)


        # Create services
        self.spawn_service = self.create_client(Spawn, 'spawn')
        self.kill_service = self.create_client(Kill, 'kill')
        self.setpen_service = self.create_client(SetPen, f'{self.turtle_name}/set_pen')

        time.sleep(2) # wait for the services to be available

    def kill_turtle(self, name: str):
        request = Kill.Request()
        request.name = name
        self.kill_service.call_async(request)
        time.sleep(1)

    def spawn_turtle(self, x, y):
        request = Spawn.Request()
        request.x = x
        request.y = y
        self.position = Position(x, y)
        request.theta = 0.0
        request.name = self.turtle_name
        self.spawn_service.call_async(request)
        time.sleep(1)

    def set_pen(self, r: int, g: int, b: int, width: int, off: int = 0):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        self.setpen_service.call_async(request)
        time.sleep(1)

    def move(self, instruction: Instruction):
        global zero_degrees, ninety_degrees, one_eighty_degrees, two_seventy_degrees

        self.publisher_.publish(instruction.export())

        # interpret instruction into a position
        self.position.add_rotation(instruction.z)
        rotation = self.position.rotation

        if rotation == zero_degrees:
            # moving horizontally to the right
            self.position.x += instruction.x
        elif rotation == ninety_degrees:
            # moving vertically up
            self.position.y += instruction.x
        elif rotation == one_eighty_degrees:
            # moving horizontally to the left
            self.position.x -= instruction.x
        elif rotation == two_seventy_degrees:
            # moving vertically down
            self.position.y -= instruction.x
        else: print("Invalid rotation value")

        print(f"Moved to position: {self.position.x}, {self.position.y} with rotation {self.position.rotation}")

        # block until the turtle has moved
        # (i hate using fixed values but i haven't studied this enough to know how to do it properly)
        time.sleep(1.5)

    def calculate_rotation(self, current_rotation: float, target_rotation: float):
        # calculate the rotation needed to reach the target rotation
        # it has to be the shortest path, so we need to check if we should rotate clockwise or counterclockwise
        if current_rotation == target_rotation:
            return 0.0

        naive_rotation = target_rotation - current_rotation

        # target rotation is in range [0, 2pi]
        # current rotation is in range [0, 2pi]
        # naive rotation is in range [-2pi, 2pi]

        # if the naive rotation is bigger than 180 (pi), we should rotate the other way
        if naive_rotation > pi:
            return naive_rotation - (2 * pi)

        # if the naive rotation is smaller than -180 (-pi), we should rotate the other way
        if naive_rotation < (-1 * pi):
            return naive_rotation + (2 * pi)

        return naive_rotation

    def move_to(self, position: Position):
        # instruction moves forward by X and rotates by Z, Z being a multiple of pi (2pi = 360) , etc etc
        # to translate that into a xyz movement, we need to calculate the distance between the current position and the target position

        # calculate the distance between the current position and the target position
        x_distance = position.x - self.position.x
        y_distance = position.y - self.position.y

        print(f"Have to move {x_distance} in x and {y_distance} in y")

        rotation = self.position.rotation

        target_rotation = zero_degrees
        rotate_by = 0.0

        # move first x then y
        if x_distance != 0.0:
            if x_distance < 0:
                # flip the direction to horizontal left
                target_rotation = one_eighty_degrees

            # rotate to the target rotation
            rotate_by = self.calculate_rotation(rotation, target_rotation)
            if rotate_by != 0.0:
                self.move(Instruction(0.0, rotate_by))
                time.sleep(0.2)

            # move the distance
            self.move(Instruction(abs(x_distance), 0.0))

        time.sleep(0.3)

        rotation = self.position.rotation
        target_rotation = ninety_degrees

        if y_distance != 0.0:
            if y_distance < 0:
                # flip the direction to vertical down
                target_rotation = two_seventy_degrees

            # rotate to the target rotation
            rotate_by = self.calculate_rotation(rotation, target_rotation)
            if rotate_by != 0.0:
                self.move(Instruction(0.0, rotate_by))
                time.sleep(0.2)

            # move the distance
            self.move(Instruction(abs(y_distance), 0.0))

        time.sleep(0.3)

    def draw_shape(self):
        self.kill_turtle("turtle1") # Kill the default turtle
        self.spawn_turtle(5.0, 5.0) # Spawn a new turtle at position (5, 5)
        self.set_pen(255, 0, 0, 2) # Set the turtle trajectory color to red

        six_positions = [
            # draw a 6
            Position(7.0, 5.0), # two to the right (start at 5,5)
            Position(7.0, 3.0), # two down
            Position(5.0, 3.0), # two to the left
            Position(5.0, 7.0), # four up
            Position(7.0, 7.0), # two to the right
        ]

        nine_positions = [
            # draw a 9
            Position(8.0, 5.0), # two down (start at 8,7)
            Position(10.0, 5.0), # two to the right
            Position(10.0, 7.0), # two up
            Position(8.0, 7.0), # two to the left
            Position(10.0, 7.0), # two to the right
            # (yes i know this repeats and i could disable pen but i'm lazy)
            Position(10.0, 3.0), # four down
            Position(8.0, 3.0), # two to the left
        ]

        # draw le six
        for position in six_positions:
            self.move_to(position)

        self.set_pen(255, 0, 0, 2, 1) # stop drawing
        self.move_to(Position(8.0, 7.0)) # space between numbers
        self.set_pen(255, 0, 0, 2, 0) # start drawing again

        # then ze nine
        for position in nine_positions:
            self.move_to(position)

        #  /$$   /$$ /$$$$$$  /$$$$$$  /$$$$$$$$
        # | $$$ | $$|_  $$_/ /$$__  $$| $$_____/
        # | $$$$| $$  | $$  | $$  \__/| $$
        # | $$ $$ $$  | $$  | $$      | $$$$$
        # | $$  $$$$  | $$  | $$      | $$__/
        # | $$\  $$$  | $$  | $$    $$| $$
        # | $$ \  $$ /$$$$$$|  $$$$$$/| $$$$$$$$
        # |__/  \__/|______/ \______/ |________/

        print("\nNice")

def main(args=None):
    rclpy.init(args=args)
    process = TurtleDraw()
    process.draw_shape()
    process.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()