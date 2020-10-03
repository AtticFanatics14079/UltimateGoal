package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class OdometryThread extends Thread implements DOThread {

    private Odometry odo;
    private ValueStorage vals;
    private double[] dimensions; //First value is trackwidth, second value is forward offset, third is auxiliary trackwidth
    private int numWheels;
    private double lastTime = 0;
    private double[] lastPos;
    private double[] lastEncoderVals;
    private double inchesPerTick = 0.0001;
    private double x = 0, y = 0, heading = 0, degTraveled = 0;

    private volatile boolean stop;

    public OdometryThread(Odometry odo, ValueStorage s, double[] dimensions) {
        this.odo = odo;
        this.vals = s;
        this.dimensions = dimensions;
        lastPos = lastEncoderVals = new double[]{0.0, 0.0, 0.0};
    }

    public OdometryThread(Odometry odo, ValueStorage s, double[] dimensions, double x, double y, double heading, double inchesPerTick) {
        this.odo = odo;
        this.vals = s;
        this.dimensions = dimensions;
        this.inchesPerTick = inchesPerTick;
        this.x = x;
        this.y = y;
        this.heading = heading;
        lastPos = lastEncoderVals = new double[]{0.0, 0.0, 0.0};
    }

    public void run() {
        while(!stop) {
            vals.waitForCycle();
            double[] encoderVals = odo.podPositions();
            if(odo.getClass() == DThreeWheelOdo.class) {
                threeWheel(encoderVals);
            }
        }
    }

    public void Stop() {
        stop = true;
    }

    public void threeWheel(double[] encoderVals) {
        //Logic to track position - probably gonna need a hand for this one, Antwon
        double leftWheelChange = (encoderVals[0] - lastEncoderVals[0]) * inchesPerTick;
        double rightWheelChange = (encoderVals[1] - lastEncoderVals[1]) * inchesPerTick;
        double frontWheelChange = (encoderVals[2] - lastEncoderVals[2]) * inchesPerTick;
        double angleChange = (leftWheelChange - rightWheelChange) / dimensions[0]; //Brad (radians)
        double leftTotal = encoderVals[0] * inchesPerTick;
        double rightTotal = encoderVals[1] * inchesPerTick;
        double rawAngle = (leftTotal - rightTotal) / dimensions[0]; //Brad
        double auxiliaryPrediction = angleChange * dimensions[2]; //Was 2, checking later
        double forward = (leftWheelChange + rightWheelChange) / 2;
        double side = frontWheelChange - auxiliaryPrediction;

        //UpdatePos

        degTraveled += Math.toDegrees(angleChange); //Doug (aka degrees)
        double sineTerm = Math.sin(angleChange) / angleChange;
        double cosTerm = (1.0 - Math.cos(angleChange)) / angleChange;
        if(Math.abs(angleChange) < 0.000001) {
            sineTerm = 1.0 - angleChange * angleChange / 6.0;
            cosTerm = angleChange / 2.0;
        }
        double move = sineTerm * forward - cosTerm * side;
        double strafe = cosTerm * forward + sineTerm * side;
        y += move;
        x += strafe;
        heading = rawAngle;

        //Part of PointDelta

        /*double c = Math.cos(heading);
        double s = Math.sin(heading);
        double newY = move * c - strafe * s;
        double newX = move * s + strafe * c;
         */
        lastEncoderVals = encoderVals;
        lastPos[0] = x;
        lastPos[1] = y;
        lastPos[2] = heading;

        odo.setPosition(new Point(x, y, heading));
    }
}
