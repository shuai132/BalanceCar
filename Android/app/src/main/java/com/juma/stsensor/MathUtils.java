package com.juma.stsensor;

import android.graphics.Point;

/**
 * Created by shuai on 2017/3/6.
 */

public class MathUtils {
    //获取两点间直线距离
    public static int getLength(float x1,float y1,float x2,float y2) {
        return (int)Math.sqrt(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2));
    }
    /**
     * 获取线段上某个点的坐标，长度为a.x - cutRadius
     * @param a 点A
     * @param b 点B
     * @param cutRadius 截断距离
     * @return 截断点
     */
    public static Point getBorderPoint(Point a, Point b, int cutRadius) {
        float radian = getRadian(a, b);
        return new Point(a.x + (int)(cutRadius * Math.cos(radian)), a.x + (int)(cutRadius * Math.sin(radian)));
    }

    //获取水平线夹角弧度
    public static float getRadian (Point a, Point b) {
        float lenA = b.x-a.x;
        float lenB = b.y-a.y;
        float lenC = (float)Math.sqrt(lenA*lenA+lenB*lenB);
        float ang = (float)Math.acos(lenA/lenC);
        ang = ang * (b.y < a.y ? -1 : 1);
        return ang;
    }
}