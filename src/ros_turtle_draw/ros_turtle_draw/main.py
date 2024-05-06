from math import pi
from turtle import pos
import rclpy
import time
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist, Vector3

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
        if rotation not in [zero_degrees, ninety_degrees, one_eighty_degrees, two_seventy_degrees]:
            raise Exception("Invalid rotation value. Must be 0, pi/2, pi, or 3pi/2")

        self.x = x
        self.y = y
        self.rotation = rotation

    def add_rotation(self, rotation: float):
        if rotation not in [zero_degrees, ninety_degrees, one_eighty_degrees, two_seventy_degrees]:
            raise Exception("Invalid rotation value. Must be 0, pi/2, pi, or 3pi/2")

        # loop around 360 degrees
        self.rotation = (self.rotation + rotation) % (2 * pi)


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
        future = self.kill_service.call_async(request)
        time.sleep(1)
        # rclpy.spin_until_future_complete(self, future)

    def spawn_turtle(self, x, y):
        request = Spawn.Request()
        request.x = x
        request.y = y
        self.position = Position(x, y)
        request.theta = 0.0
        request.name = self.turtle_name
        future = self.spawn_service.call_async(request)
        time.sleep(1)
        # rclpy.spin_until_future_complete(self, future)

    def set_pen(self, r: int, g: int, b: int, width: int, off: int = 0):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        future = self.setpen_service.call_async(request)
        time.sleep(1)
        # rclpy.spin_until_future_complete(self, future)

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

        # block until the turtle has moved (i hate using fixed values but i haven't studied this enough to know how to do it properly)
        time.sleep(1)

    def move_to(self, position: Position):
        # instruction moves forward by X and rotates by Z, Z being a multiple of pi (2pi = 360) , etc etc
        # to translate that into a xyz movement, we need to calculate the distance between the current position and the target position

        # calculate the distance between the current position and the target position
        x_distance = position.x - self.position.x
        y_distance = position.y - self.position.y

        print(f"Have to move {x_distance} in x and {y_distance} in y")

        rotation = self.position.rotation

        # move first x then y
        if x_distance != 0.0:
            if rotation == ninety_degrees:
                # moving vertically up, change to horizontal right
                self.move(Instruction(0.0, two_seventy_degrees))
            elif rotation == one_eighty_degrees:
                # moving horizontally to the left, change to horizontal right
                self.move(Instruction(0.0, one_eighty_degrees))
            elif rotation == two_seventy_degrees:
                # moving vertically down, change to horizontal right
                self.move(Instruction(0.0, ninety_degrees))

            if x_distance < 0:
                # flip the direction to horizontal left
                self.move(Instruction(0.0, one_eighty_degrees))

            # move the distance
            return self.move(Instruction(abs(x_distance), 0.0))

        rotation = self.position.rotation

        if y_distance != 0.0:
            if rotation == zero_degrees:
                # moving horizontally to the right, change to vertical up
                self.move(Instruction(0.0, ninety_degrees))
            elif rotation == one_eighty_degrees:
                # moving horizontally to the left, change to vertical up
                self.move(Instruction(0.0, two_seventy_degrees))
            elif rotation == two_seventy_degrees:
                # moving vertically down, change to vertical up
                self.move(Instruction(0.0, one_eighty_degrees))

            if y_distance < 0:
                # flip the direction to vertical down
                self.move(Instruction(0.0, one_eighty_degrees))

            # move the distance
            return self.move(Instruction(abs(y_distance), 0.0))

    def draw_shape(self):
        self.kill_turtle("turtle1") # Kill the default turtle
        self.spawn_turtle(5.0, 5.0) # Spawn a new turtle at position (5, 5)
        self.set_pen(255, 0, 0, 2) # Set the turtle trajectory color to red

        six_positions = [
            # draw a 6
            Position(7.0, 5.0),
            Position(7.0, 3.0),
            Position(5.0, 3.0),
            Position(5.0, 7.0),
            Position(7.0, 7.0),
        ]

        nine_positions = [
            # draw a 9
            Position(8.0, 5.0),
            Position(10.0, 5.0),
            Position(10.0, 7.0),
            Position(8.0, 7.0),
            Position(10.0, 7.0),
            Position(10.0, 3.0),
            Position(8.0, 3.0),
        ]

        for position in six_positions:
            self.move_to(position)

        self.set_pen(255, 0, 0, 2, 1) # stop drawing
        self.move_to(Position(8.0, 7.0)) # space between numbers
        self.set_pen(255, 0, 0, 2, 0) # start drawing again

        for position in nine_positions:
            self.move_to(position)

def main(args=None):
    rclpy.init(args=args)


    process = TurtleDraw()
    process.draw_shape()

    rclpy.spin(process)

    process.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()