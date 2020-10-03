package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class Point{

    public double x;
    public double y;
    public double heading;

    Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    Point(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.heading = angle;
    }

    public Point add(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public Point subtract(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public Point multiply(double scaler) {
        return new Point(x * scaler, y * scaler);
    }

    public Point divide(double scaler) {
        return new Point(x / scaler, y / scaler);
    }
    public Point normalize(Point vector, double hypotenuse){
        return vector.divide(hypotenuse);
    }

    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }
}
