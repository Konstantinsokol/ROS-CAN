#!/usr/bin/python3
from robot_msgs.srv import Service_Ang_Len, Service_Ang_LenResponse
import rospy
from geometry_msgs.msg import Point32
from setting_pkg.const import *
import math
from robot_msgs.msg import peers
from dataclasses import dataclass


@dataclass
class tag():
    id: str
    x: float = None
    y: float = None


backtag = tag("c30c")
fronttag = tag("49ad")
tabletag = tag("00b4")


def Response():

    while not all((fronttag.x, fronttag.y, backtag.x, backtag.y, tabletag.x, tabletag.x)):
        rospy.sleep(.1)
    # Координаты точек
    front_x = fronttag.x
    front_y = fronttag.y

    back_x = backtag.x
    back_y = backtag.y

    table_x = tabletag.x
    table_y = tabletag.y

    # длины сторон треугольника в метрах
    length_table_front = math.sqrt(
        (table_x - front_x)**2 + (table_y - front_y)**2)
    length_table_back = math.sqrt(
        (table_x - back_x)**2 + (table_y - back_y)**2)
    length_front_back = 2

    # находим углы треугольника в градусах
    # angle_a = math.degrees(math.acos((length_table_back**2 + length_front_back**2 - length_front_back**2**2) / (2 * length_table_back * length_front_back)))
    angle_ = math.degrees(math.acos((length_table_front**2 + length_front_back**2 - length_table_back**2) /
                                    (2 * length_table_front * length_front_back)))
    # angle_c = math.degrees(math.acos((length_table_front**2 + length_table_back**2 - length_front_back**2) / (2 * length_table_front * length_table_back)))

    # Функция по созданию ответа в виде угла и дистанции

    # Проверка, что точки лежат на одной прямой
    if (table_x - front_x) * (back_y - front_y) == (table_y - front_y) * (back_x - front_x):
        check = True

    else:
        check = False
    # уравнение прямой через три точки

    if check:
        # расчёт для движения, когда все три точки на одной прямой
        if length_table_front < length_table_back:
            res = (0, length_table_front)
            return res

        elif length_table_front > length_table_back:
            res = (180, length_table_front)
            return res

        elif abs(length_table_back) < 1.3:
            res = (0, -1)
            return res

    else:
        if table_x > front_x:
            if front_y > back_y:
                # Угол положителен, вращение по часовой стрелке
                res = (180 - angle_, length_table_front)
                return res

            else:
                # Угол отрицателен, вращение против часовой стрелки
                res = (angle_ - 180, length_table_front)
                return res

        else:
            if front_y > back_y:
                # Угол отрицателен, вращение против часовой стрелки
                res = (angle_ - 180, length_table_front)
                return res

            else:
                # Угол положителен, вращение по часовой стрелке
                res = (180 - angle_, length_table_front)
                return res


def handel_peers(data: any):
    rospy.loginfo(
        f" ID:{data.Id}, x:{data.coordinate.x}, y:{data.coordinate.y}")
    if data.Id == fronttag.id:
        fronttag.x, fronttag.y = data.coordinate.x, data.coordinate.y
    elif data.Id == backtag.id:
        backtag.x, backtag.y = data.coordinate.x, data.coordinate.y
    elif data.Id == tabletag.id:
        tabletag.x, tabletag.y = data.coordinate.x, data.coordinate.y


def handle_angle_len(data):
    if data.command == 1:
        rospy.loginfo(data)
        res = Service_Ang_LenResponse()
        angle_, length_ = Response()
        res.angle = int(angle_)
        res.length = float(length_)
        return res


def angl_len_server():
    rospy.init_node(ROBO_PATHPLANING_NODE_NAME)
    rospy.Subscriber(
        ROBO_NAVIGATION_TOPIC_TH_NAME, peers, handel_peers)
    rospy.loginfo(
        f"server")
    rospy.Service(ROBO_NAVIGATION_SERVICE_TH_NAME,
                  Service_Ang_Len, handle_angle_len)

    rospy.spin()


if __name__ == "__main__":
    angl_len_server()
