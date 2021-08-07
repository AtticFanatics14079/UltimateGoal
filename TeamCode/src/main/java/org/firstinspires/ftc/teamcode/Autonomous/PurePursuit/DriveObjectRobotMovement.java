package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DMotor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;

import java.util.ArrayList;


public class DriveObjectRobotMovement {
    public Point robotPosition = new Point(0,0);//NEED TO UPDATE ROBOT WORLD POSITION
    public double worldAngle = 0;  //NEED TO UPDATE WORLD ANGLE
    public DMotor[] Motors = new DMotor[4];
    private CurvePoint[] pointsInReference = null;
    private int targetPoint = 1;

    //add edge cases
    public DriveObjectRobotMovement(HardwareMap hwMap){

        Motors[0] = new DMotor(hwMap, "back_left_motor");
        Motors[1] = new DMotor(hwMap, "front_left_motor");
        Motors[2] = new DMotor(hwMap, "front_right_motor");
        Motors[3] = new DMotor(hwMap, "back_right_motor");

        Motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        Motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        Motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DriveObjectRobotMovement(ConfigurationRR config){
        Motors[1] = config.motors.get(0);
        Motors[0] = config.motors.get(1);
        Motors[3] = config.motors.get(2);
        Motors[2] = config.motors.get(3);
        System.out.println(config.hardware.size());
    }

    public boolean followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

        //Removes excessive points from reference
        if(pointsInReference == null) {
            pointsInReference = new CurvePoint[2];
            pointsInReference[0] = allPoints.get(0);
            pointsInReference[1] = allPoints.get(1);
        }
        System.out.println("Hypotenuse: " + Math.hypot(robotPosition.x - pointsInReference[1].x, robotPosition.y - pointsInReference[1].y));
        System.out.println("Robot pose: " + robotPosition);
        if(targetPoint < allPoints.size()-1 && Math.hypot(robotPosition.x - pointsInReference[1].x, robotPosition.y - pointsInReference[1].y) < pointsInReference[1].followDistance-(2*pointsInReference[1].moveSpeed)) { //Arbitrary number multiplied
            targetPoint++;
            pointsInReference[0] = pointsInReference[1];
            pointsInReference[1] = allPoints.get(targetPoint);
            System.out.println("Target point: " + targetPoint);
        }

        Point robotPoint = new Point(robotPosition.x,robotPosition.y);
        int shortPoint = Geometry.distanceBetweenPoints(robotPoint, pointsInReference[0].toPoint()) < Geometry.distanceBetweenPoints(robotPoint,pointsInReference[1].toPoint()) ? 0 : 1;
        CurvePoint followMe = getFollowPointPath(pointsInReference, robotPoint, pointsInReference[shortPoint].followDistance);
        System.out.println("ShortPoint:" + shortPoint);
        //End of new stuff

        System.out.println("Going Towards Point: " + followMe);
        double speed = followMe.moveSpeed;
        if(targetPoint == allPoints.size()-1 && Math.hypot(robotPosition.x-allPoints.get(allPoints.size()-1).x,robotPosition.y-allPoints.get(allPoints.size()-1).y)<allPoints.get(0).followDistance){
            followMe = allPoints.get(allPoints.size()-1);
            speed = Range.clip(Math.hypot(robotPosition.x - followMe.x, robotPosition.y - followMe.y) / 4.0, -1, 1) * allPoints.get(allPoints.size() - 1).moveSpeed;
        }
        System.out.println(followMe.toString());
        goToPosition(followMe.x, followMe.y, speed, followAngle, followMe.turnSpeed);
        if(targetPoint == allPoints.size()-1 && Math.hypot(robotPosition.x-allPoints.get(allPoints.size()-1).x,robotPosition.y-allPoints.get(allPoints.size()-1).y)<1){
            setPower(0,0,0);
            return true;
        }
        else{
            return false;
        }
        //can go to op mode and run it
    }

    public CurvePoint getFollowPointPath(CurvePoint[] pathPoints, Point robotLocation, double followRadius) {

        CurvePoint followMe = new CurvePoint(pathPoints[1]); //default go to very second point
        System.out.println("FollowMeBeforeIntersections: " + followMe.toString());
        //Can clean this up if my previous stuff works.
        ArrayList<Point> intersections = Geometry.lineCircleIntersection(robotLocation, followRadius, pathPoints[0].toPoint(), pathPoints[1].toPoint());
        System.out.println(intersections);
        double closestAngle = 1000;

        for(Point thisIntersection : intersections){
            System.out.print("ThisIntersection: " + thisIntersection);
            double angle = Math.atan2(thisIntersection.y - robotPosition.y, thisIntersection.x - robotPosition.x); // absolute angle to world coordinate space
            double deltaAngle = Math.abs(AngleFunction.AngleWrap(angle - worldAngle)); //his code had _rad after worldAngle

            if(deltaAngle < closestAngle) {
                closestAngle = deltaAngle;
                followMe.setPoint(thisIntersection);
            }
            // if angle is the same returns zero, otherwise corrects
        }
        System.out.println("FollowMeBeforeReturn: " + followMe.toString());
        return followMe;
    }

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
        Point point1 = new Point(x,y);
        double distanceToTarget = Geometry.distanceBetweenPoints(robotPosition, point1);

        double absoluteAngleToTarget = AngleFunction.turnAngle(point1, robotPosition)+Math.toRadians(90);

        double relativeAngleToPoint = AngleFunction.AngleWrap(absoluteAngleToTarget - worldAngle);


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint)); //guratness movement x power to be from 0 -1
        double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint)+ Math.abs(relativeYToPoint));

        // import our actual motors functions
        double movement_x = movementXPower * movementSpeed;
        double movement_y = movementYPower * movementSpeed;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + Math.toRadians(preferredAngle);
        double movement_Turn = Range.clip(relativeTurnAngle / Math.toRadians(90), -1, 1) * turnSpeed; //need turning function and import range clip
        if(distanceToTarget < 5) {
            movement_Turn = 0;
        }
        setPower(-movement_x, -movement_y, -movement_Turn);
    }

    public void updatePose(Point pos, double worldAng){
        this.robotPosition = pos;
        this.worldAngle = worldAng;
    }

    public void setPower(double px, double py, double pa){ //Multiplied pa by -1 to suit turning requests
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        Motors[0].setPower(p1);
        Motors[1].setPower(p2);
        Motors[2].setPower(p3);
        Motors[3].setPower(p4);
    }

}


