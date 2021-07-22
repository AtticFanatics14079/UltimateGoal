package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;
import java.lang.reflect.Array;
import java.util.ArrayList;

public class Geometry {
    public static double distanceBetweenPoints(Point point1, Point point2) {

        double hypotenuse = Math.hypot(point1.x - point2.x, point1.y - point2.y);
        return hypotenuse;
    }

    public static Point[] spacePoints(Point startPoint, Point endPoint, double Spacing) {
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

        return pointsArray;
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

    public static ArrayList<CurvePoint> createArc(Point center, CurvePoint start, CurvePoint end, double numPoints, boolean shortArc) {
        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(start);

        //First we find out the angle interval between points.
        double a = Math.hypot(start.x-center.x, start.y-center.y), c = Math.hypot(start.x-end.x, start.y-end.y); //a is the radius.

        double translateX = (start.x - center.x) / a, translateY = (start.y - center.y) / a;
        double startAngle = Math.acos((2-Math.hypot(translateX, translateY - 1))/2) * (translateX < 0 ? -1 : 1); //Finds the angle between start and (1, 0) and makes negative if it is.

        double endTranX = (end.x - center.x) / a, endTranY = (end.y - center.y) / a;
        double endAngle = Math.acos((2-Math.hypot(endTranX, endTranY - 1))/2) * (endTranX < 0 ? -1 : 1); //Finds the angle between end and (1, 0) and makes negative if it is.

        double angle = endAngle - startAngle;
        if(!shortArc) angle = (2 * Math.PI - Math.abs(angle)) * Math.abs(angle) / angle; //Changes to longer arc
        double angleStep = angle / numPoints; //Interval in radians between each point. Do something to allow for longer arc.

        //Next step is to determine where start would be in the unit circle, where center is at (0, 0).


        //System.out.println("Angle step * numPoints: " + angleStep * numPoints + ", Start angle: " + startAngle);
        //System.out.println("Angle step check: " + Math.abs(startAngle + angleStep * numPoints - angle - 1));

        //System.out.println("Start angle: " + startAngle + ", angle: " + angle);
        //System.out.println("Angle step check: " + Math.abs(Math.sin(endAngle)*a+center.x - end.x) + " End angle: " + endAngle);

        //if(Math.abs(Math.cos(startAngle + angle)) > 0.001) angleStep *= -1; //Makes sure the angle step goes from start to end.

        for(int i = 1; i <= numPoints; i++) {
            CurvePoint newPoint = new CurvePoint(end); //Keeps other variables from end.
            newPoint.setPoint(new Point(a * Math.sin(startAngle + i * angleStep) + center.x, a * Math.cos(startAngle + i * angleStep) + center.y));
            points.add(newPoint);
        }

        points.add(end);
        
        return points;
    }
}