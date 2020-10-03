package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

public class AngleFunction {
    public static double turnAngle(Point point1, Point point2) {

        double turnAngle = Math.atan2(point2.gety() - point1.gety(), point2.getx() - point1.getx());

        return turnAngle;
    }
    public static double AngleWrap(double angle) { //makes sure angle is within range -180 to 180 degrees
        while(angle < -Math.PI) {
            angle += 2*Math.PI;
        }
        while(angle > +Math.PI) {
            angle -= 2*Math.PI;
        }
        return angle;
    }
}