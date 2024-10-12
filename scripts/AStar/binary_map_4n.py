#!/usr/bin/env python3
from AStar.gridmap import OccupancyGridMap
from AStar.a_star import a_star
import math
import png
import os
def path_planing(start_node,goal_node,numbers=0):

    gmap = OccupancyGridMap.from_png("/home/abd/ros_ws/src/my_robot_controll/scripts/AStar/ssss.png", 0.01)


    path, path_px = a_star(start_node, goal_node, gmap, movement='4N',meter=0)

    
    def get_angle(x1,y1,x2,y2):
        return math.degrees(math.atan2(y2-y1,x2-x1))

    angles = []
    points = []
    final_path = []
    final_path.append(path_px[0])

    if numbers == 0:
        for i in range (0,len(path_px)-1):
            x1 = path_px[i][0]
            y1 = path_px[i][1]
            x2 = path_px[i+1][0]
            y2 = path_px[i+1][1]

            angles.append(get_angle(x1,y1,x2,y2))
            points.append(((x1,y1),(x2,y2)))

        angles.append(200)

        for i in range (0,len(angles)-1):
            if angles[i] != angles[i+1]:
                final_path.append(points[i][1])


    else:
        numbers = numbers+1
        for i in range (0,len(path_px)-1):
            x1 = path_px[i][0]
            y1 = path_px[i][1]
            x2 = path_px[i+1][0]
            y2 = path_px[i+1][1]

            angles.append(get_angle(x1,y1,x2,y2))
            points.append(((x1,y1),(x2,y2)))

        angles.append(200)

        for i in range (0,len(angles)-1):
            if angles[i] != angles[i+1]:
                v1 = final_path[-1:][0]        
                p1 = path_px.index(v1)
                p2 = i+1

                for j in range (0,numbers):
                    p = p2-p1
                    # if j == numbers-1:
                    #     continue
                    final_path.append(path_px[p1:p2:round(p/numbers)][j])
                final_path.append(path_px[i+1])


    final_path = list(dict.fromkeys(final_path))
    return final_path,path_px


#هاد هو التابع يلي بهمك من كل هالكود
#وقت بدك النقاط يلي موجودة عند كل انحناء استخدم هاد التابع




#و وقت بدك تحدد عدد من النقاط بين كل انحنائين استخدم هاد الكود 
#  (#) ولا تنسى انك وقت بدك تختار واحد من هدول التابعين انك تهمش التابع التاني , يعني قصدي تحط قبله اشارة   

a,b=path_planing((0,0),(200,93))

# a : برجعلك مجموعة النقاط يلي بدك اياها حسب كل تابع
# b : برجعلك كل النقاط من نقطة البداية للنقطة الهدف بدون مايحذف ولا نقطة , يعني برجعلك المسار كامل

# هاد الملف بيرسملك الخريطة وبحدد المسار عليها , binary_map_4n_2 في ملف اسمه
# يعني بعد ما تجيب النقاط من هاد الكود , فيك تروح على هداك الكود وترسم الخريطة والمسار وهيك بكون الموضوع اوضح بالنسبة الك
#وبهداك الكود كمان في عندك تابع واحد عند الاخير , بس حط فيه نقطة البداية والنهاية يلي حطيتها بهاد الكود


# print('\n',b)