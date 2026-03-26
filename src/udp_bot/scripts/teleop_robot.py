#!/usr/bin/env python3
import rospy
import sys
import termios
import tty
from select import select
from udp_bot.msg import kirim_kecepatan_udp

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

servo_ang = 0.0;

def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
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


def send_teleop_callback(event):
    # rospy.loginfo("vx: %f, vy: %f, vw: %f", msg.vx, msg.vy, msg.vw)
    msg = kirim_kecepatan_udp()
    msg.kecepatan_x = velx
    msg.kecepatan_y = vely
    msg.kecepatan_sudut = velw
    msg.sudut_servo = servo_ang
    velocity_pub.publish(msg)
    # rospy.loginfo("ini timer")

if __name__ == '__main__':
    rospy.init_node("teleop_robot")
    
    # Create a ROS publisher
    velocity_pub = rospy.Publisher("kecepatan_kirim_udp", kirim_kecepatan_udp, queue_size=10)
    rospy.Timer(rospy.Duration(0.01), send_teleop_callback)

    settings = saveTerminalSettings()
    print(msg_teleop)
    try:
        while(1):
            key = getKey(settings, 0.5)
            if key == 'w':
                # print("maju")
                velx = 0.0
                vely = 1 * speed
                velw = 0.0
            elif key == "s":
                # print("mundur")
                velx = 0.0
                vely = -1 * speed
                velw = 0.0
            elif key == "a":
                # print("kiri")
                velx = -1 * speed
                vely = 0.0
                velw = 0.0
            elif key == "d":
                # print("kanan")
                velx = 1 * speed
                vely = 0.0
                velw = 0.0
            elif key == "q":
                # print("putar kiri")
                velx = 0.0
                vely = 0.0
                velw = 3 * speed
            elif key == "e":
                # print("putar kanan")
                velx = 0.0
                vely = 0.0
                velw = -3 * speed
            elif key == "p":
                speed += 0.01
                if speed > 0.5:
                    speed = 0.5
                # print("speed: ", speed)
            elif key == "o":
                speed -= 0.01
                if speed < 0.01:
                    speed = 0.01
                # print("speed: ", speed)
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
                if (key == '\x03'):
                    break
    except Exception as e:
        print(e)

    finally:
        restoreTerminalSettings(settings)

    rospy.spin()