#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from second_coursework.srv import GetRoomCoordRequest, GetRoomCoordResponse, GetRoomCoord

class RoomService():
    def __init__(self) -> None:
        self.get_room_coord_srv = rospy.Service('get_coord', GetRoomCoord, self.get_coord)
        # in all of these value of z = 0
        self.index_A = 0
        self.room_a_points = [(0.7, 10), (0.7, 6.6), (3.2, 6.6), (3.2, 10)]

        self.index_B = 0
        self.room_b_points = [(4.4, 10), (4.4, 6.6), (7.7, 6.6), (7.7, 10)]

        self.index_C = 0
        self.room_c_points = [(8.9, 10), (8.9, 6.6), (12.4, 6.6), (12.4, 10)]

        self.index_D = 0
        self.room_d_points = [(0.7, 5.4), (0.7, 0.6), (3.2, 0.6), (3.2, 5.2)]

        self.index_E = 0
        self.room_e_points = [(4.5, 5.1), (4.5, 0.6), (7.6, 0.6), (7.5, 5.2)]

        self.index_F = 0
        self.room_f_points = [(9, 4.1), (9, 0.6), (12, 0.6), (12, 4.1)]

    def get_coord(self, req: GetRoomCoordRequest):
        point = Point()
        if req.room_name == "A":
            point.x = self.room_a_points[self.index_A % 4][0]
            point.y = self.room_a_points[self.index_A % 4][1]
            self.index_A += 1

        if req.room_name == "B":
            point.x = self.room_b_points[self.index_B % 4][0]
            point.y = self.room_b_points[self.index_B % 4][1]
            self.index_B += 1

        if req.room_name == "C":
            point.x = self.room_c_points[self.index_C % 4][0]
            point.y = self.room_c_points[self.index_C % 4][1]
            self.index_C += 1

        if req.room_name == "D":
            point.x = self.room_d_points[self.index_D % 4][0]
            point.y = self.room_d_points[self.index_D % 4][1]
            self.index_D += 1

        if req.room_name == "E":
            point.x = self.room_e_points[self.index_E % 4][0]
            point.y = self.room_e_points[self.index_E % 4][1]
            self.index_E += 1

        if req.room_name == "F":
            point.x = self.room_f_points[self.index_F % 4][0]
            point.y = self.room_f_points[self.index_F % 4][1]
            self.index_F += 1

        point.z = 0

        return GetRoomCoordResponse(point)        

if __name__ == '__main__':
    rospy.init_node('roomservice')
    get_room_coord_srv = RoomService()
    rospy.spin()