package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public interface Odometry extends DriveObject {

    Point getPosition();
    void setPosition(Point newPos);
    double[] podPositions();
    double[] get();
    double getX();
    double getY();
    double getHeading();
    void beginTracking();
    void endTracking();
}
