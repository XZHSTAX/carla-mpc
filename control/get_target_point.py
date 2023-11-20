
import numpy as np
import math

def get_intersection(polyline,index,lookahead):
        # 获得跨越圆的两个点的坐标
        x1,y1 = polyline[index]
        x2,y2 = polyline[index+1]

        # 获得过这两点的直线方程
        a = (y2 - y1)/(x2 - x1)
        b = (x2*y1 - x1*y2)/(x2 - x1)

        if( a**2*lookahead**2 - b**2 +lookahead**2 < 0 ): # 如果无解，即没有交点
            return None
        else:
            # 圆的方程和直线方程联立求解，获得坐标
            x = (-a*b + math.sqrt(a**2*lookahead**2 - b**2 +lookahead**2)) / (a**2 + 1)
            
            if (x < x1 or x > x2): # 检查此解不在线段内
                x = (-a*b - math.sqrt(a**2*lookahead**2 - b**2 +lookahead**2)) / (a**2 + 1) # 换个解
                if (x < x1 or x > x2): # 如果还是不在线段内
                    return None
            y = a*x + b

            coordinate = [x,y]
            return coordinate

def get_target_point(lookahead, polyline,return_index = 0):
    """ Determines the target point for the pure pursuit controller
    
    Parameters
    ----------
    lookahead : float
        The target point is on a circle of radius `lookahead`
        The circle's center is (0,0)
    poyline: array_like, shape (M,2)
        A list of 2d points that defines a polyline.
    
    Returns:
    --------
    target_point: numpy array, shape (,2)
        Point with positive x-coordinate where the circle of radius `lookahead`
        and the polyline intersect. 
        Return None if there is no such point.  
        If there are multiple such points, return the one that the polyline
        visits first.
    """
    # Hint: A polyline is a list of line segments. 
    # The formulas for the intersection of a line segment and a circle are given
    # here https://mathworld.wolfram.com/Circle-LineIntersection.html
    # raise NotImplementedError

    # coordinate_list = [get_intersection(polyline,index,lookahead) for index in range(polyline.shape[0])] 
    i = 0
    if( polyline.shape[1] ==2 ):
        for index in range(polyline.shape[0]-1):
            coordinate = get_intersection(polyline,index,lookahead)
            if(coordinate!=None and coordinate[0]>0):
                i = index
                break

    # 如果轨迹信息里含有 yaw
    elif( polyline.shape[1] ==3):
        for index in range(polyline.shape[0]-1):
            coordinate = get_intersection(polyline[:,:2],index,lookahead)
            if(coordinate!=None and coordinate[0]>0):
                coordinate.append(polyline[index,2])
                i = index
                break

    if( return_index):
        return coordinate,i
    else:
        return coordinate



if __name__ == '__main__':
    a = get_target_point(5,np.array([[1,1], [2,3], [3,6], [4,7]]))
    print(a)