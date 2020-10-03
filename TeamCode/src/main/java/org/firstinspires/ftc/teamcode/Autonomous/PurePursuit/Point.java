package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;


public class Point{
    double x;
    double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getx() {
        return x;
    }

    public double gety() {
        return y;
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
