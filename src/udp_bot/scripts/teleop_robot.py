#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import termios
import tty
import threading
from select import select
from udp_bot.msg import KirimKecepatanUdp

msg_teleop = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d

no keypress = stop

w/s : forward/backward
a/d : left/right
q/e : rotate left/rotate right
p/o : increase/decrease speeds
i/k : increase/decrease servo angle

CTRL-C to quit
"""

velx = 0.0
vely = 0.0
velw = 0.0
speed = 0.02

servo_ang = 0.0


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    return termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class TeleopRobot(Node):
    def __init__(self):
        super().__init__('teleop_robot')
        self.velocity_pub = self.create_publisher(KirimKecepatanUdp, 'kecepatan_kirim_udp', 10)
        self.timer = self.create_timer(0.01, self.send_teleop_callback)

    def send_teleop_callback(self):
        msg = KirimKecepatanUdp()
        msg.kecepatan_x = velx
        msg.kecepatan_y = vely
        msg.kecepatan_sudut = velw
        msg.sudut_servo = servo_ang
        self.velocity_pub.publish(msg)


def main(args=None):
    global velx, vely, velw, speed, servo_ang

    rclpy.init(args=args)
    node = TeleopRobot()

    # Spin in a separate thread so the timer callback runs
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = saveTerminalSettings()
    print(msg_teleop)
    try:
        while True:
            key = getKey(settings, 0.5)
            if key == 'w':
                velx = 0.0
                vely = 1 * speed
                velw = 0.0
            elif key == "s":
                velx = 0.0
                vely = -1 * speed
                velw = 0.0
            elif key == "a":
                velx = -1 * speed
                vely = 0.0
                velw = 0.0
            elif key == "d":
                velx = 1 * speed
                vely = 0.0
                velw = 0.0
            elif key == "q":
                velx = 0.0
                vely = 0.0
                velw = 3 * speed
            elif key == "e":
                velx = 0.0
                vely = 0.0
                velw = -3 * speed
            elif key == "p":
                speed += 0.01
                if speed > 0.5:
                    speed = 0.5
            elif key == "o":
                speed -= 0.01
                if speed < 0.01:
                    speed = 0.01
            elif key == "i":
                servo_ang += 2.0
                if servo_ang > 90:
                    servo_ang = 90
            elif key == "k":
                servo_ang -= 2.0
                if servo_ang < -90:
                    servo_ang = -90
            else:
                velx = 0.0
                vely = 0.0
                velw = 0.0
                if key == '\x03':
                    break
    except Exception as e:
        print(e)
    finally:
        restoreTerminalSettings(settings)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
