import rclpy
from rclpy.node import Node
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath
from enum import Enum, auto
from polyglotbot_interfaces.srv import StringToWaypoint
import numpy as np
from geometry_msgs.msg import Point

class State(Enum):
    SERVICE = auto(),

class CreateWaypoint(Node):
    def __init__(self):
        super().__init__("create_waypoint")

        #Service
        self.waypoint_srv = self.create_service(StringToWaypoint, 'string2waypoint', self.string2waypoint_callback)

    def string2waypoint_callback(self, request, response):
        self.string = request.text
        self.language = request.language
        self.wp = self.str2waypoint(self.string, self.language)
        print("Here 1")
        self.mess = self.create_waypoints_message(self.wp)
        print("Here 3")
        # print(self.rez)
        response.waypoints = self.mess
        print(response.waypoints)
        print("Here 4")
        print(response)

        return response

    def create_waypoints_message(self, waypoints):
        # point_messages = [Point(x=x, y=y, z=z) for x, y, z in waypoints]
        print("Here 2")
        point_messages = []
        for i in waypoints:
            point_messages.append(Point(x=i[0], y=i[1], z=i[2]))

        return point_messages

    def str2waypoint(self, string, language):
        #Prepare for different languages
        #Need to upload to main package and create package share paths
        if language == 'zh-CN':
            fontPath = "/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_SC/NotoSansSC-VariableFont_wght.ttf"
        elif language == 'zh-TW':
            fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_TC/NotoSansTC-VariableFont_wght.ttf"
        elif language == 'ja':
            fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_JP/NotoSansJP-VariableFont_wght.ttf"
        elif language == 'ko':
            fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_KR/NotoSansKR-VariableFont_wght.ttf"
        elif language == 'iw' or language == 'yi':
            fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Noto_Sans_Hebrew/NotoSansHebrew-VariableFont_wdth,wght.ttf"
        else:
            fontPath="/home/kashedd/fpws/src/fp/final-project-dkoh555/svg/Fonts/Roboto/Roboto-Thin.ttf"

        #Create FontProperties object with "custom" google font
        customFont = FontProperties(fname=fontPath)

        startX, startY = 0, 0

        for char in string:
            if char.isspace():
                continue
            #Path of current character
            charPath = TextPath((startX, startY), char, size=1., prop=customFont)
            #Get vertices of path
            charVerts = charPath.vertices
            #Remove unnecessary points
            charVerts = charVerts[~(charVerts[:,0]%1<0.01)]
            # print(charVerts.shape)

            duplicate_entry = []

            for i in range(0, len(charVerts)):
                for j in range(i + 1, len(charVerts)):
                    if np.array_equal(charVerts[i], charVerts[j]):
                        duplicate_entry.append(charVerts[j])

            # Find indices of duplicate entries
            indices_of_duplicates = []
            for duplicate in duplicate_entry:
                indices = [index for index, cv in enumerate(charVerts) if np.array_equal(cv, duplicate)]
                indices_of_duplicates.append(indices)

            ind = []
            for i, indices in enumerate(indices_of_duplicates):
                ind += [indices[1]+1]

            a = np.split(charVerts, ind)
            # print(a.shape)
            a = [arr for arr in a if arr.size > 0]
            # print(a.shape)
            a = [np.column_stack((arr, np.full(arr.shape[0], 0))) for arr in a]

            # print(a.shape)
            # print(a)
            test = []
            for i in range(len(a)):
                for j in range(len(a[i])):
                    if i is not 0 and j is 0:
                        x = a[i][j][0]
                        y = a[i][j][1]
                        z = 1.0
                        xyz = [x, y, z]
                        test.append(xyz)
                        x = a[i][j][0]
                        y = a[i][j][1]
                        z = -1.0
                        xyz = [x, y, z]
                        test.append(xyz)


                    x = a[i][j][0]
                    y = a[i][j][1]
                    z = a[i][j][2]
                    xyz = [x, y, z]
                    test.append(xyz)

                    if i is not (len(a) - 1) and j is (len(a[i]) - 1):
                        x = a[i][j][0]
                        y = a[i][j][1]
                        z = 1.0
                        xyz = [x, y, z]
                        test.append(xyz)


            # for i in range(len(a)):
            #     if i+1 < len(a):
            #         a[i][-1][2] = 1
            #         x = a[i][-1][0]
            #         y = a[i][-1][1]
            #         z = a[i][-1][2]
            #         xyz = [x, y, z]
            #         test.append(xyz)
  
            #         a[i+1][0][2] = 1
            #         x = a[i+1][0][0]
            #         y = a[i+1][0][1]
            #         z = a[i+1][0][2]
            #         xyz = [x, y, z]
            #         test.append(xyz)
                    
            #         a[i+1][0][2] = -1
            #         x = a[i+1][0][0]
            #         y = a[i+1][0][1]
            #         z = a[i+1][0][2]
            #         xyz = [x, y, z]
            #         test.append(xyz)
                    
                    

                # else:
                #     pass
            print(a)
            print(test)
            return test
           
    



def main(args=None):
    rclpy.init(args=args)
    node = CreateWaypoint()
    rclpy.spin(node)
    rclpy.shutdown()

# if __name__ == '__main__':
#     # text = "eo"
#     # lan = 'zh-CN'
#     # str2waypoint(text, lan)
#     main()