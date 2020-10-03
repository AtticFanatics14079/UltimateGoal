package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;
import java.util.ArrayList;

public class Geometry {
    public static double distanceBetweenPoints(Point point1, Point point2) {

        double hypotenuse = Math.hypot(point1.x - point2.x, point1.y - point2.y);
        return hypotenuse;
    }

    public static void spacePoints(Point startPoint, Point endPoint, double Spacing) {
        Point vector = startPoint.subtract(endPoint);
        Point[] pointsArray;
        double pointsThatFit = Math.ceil(distanceBetweenPoints(startPoint, endPoint) / Spacing);
        pointsArray = new Point[(int) pointsThatFit + 1];
        Point vector_normalized = vector.normalize(vector, distanceBetweenPoints(startPoint, endPoint));
        vector_normalized = vector_normalized.multiply(Spacing);
        for (int i = 0; i < pointsThatFit; i++) {
            pointsArray[i] = startPoint.add(vector_normalized.multiply(i));
        }
        pointsArray[(int) pointsThatFit] = endPoint;

    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        //modify difference if too small
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }
        //slope
        double m1 = (linePoint1.y - linePoint2.y)/(linePoint1.x - linePoint2.x);
        //shift points to origin
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        //quadratics
        double quadraticA = 1.0 + Math.pow(m1, 2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = (Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {

            //Quadratic equation
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - 4.0 * quadraticA * quadraticC)) / 2.0 * quadraticA;
            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - 4.0 * quadraticA * quadraticC)) / 2.0 * quadraticA;

            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            //Add back offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            //Find min and max of x points of line segment, ? operator is mini if() where after ? is if true and after : is if false
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            //Check if sure on line
            if(xRoot1 > minX && xRoot1 < maxX) {
                Point root = new Point(xRoot1, yRoot1);
                allPoints.add(root);
            }
            if(xRoot2 > minX && xRoot2 < maxX) {
                Point root = new Point(xRoot2, yRoot2);
                allPoints.add(root);
            }
        }catch(Exception e) {

        }
        return allPoints;
    }
}
