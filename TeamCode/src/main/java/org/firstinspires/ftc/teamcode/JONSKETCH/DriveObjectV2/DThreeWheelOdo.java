/* THIS CLASS WILL LIKELY NOT BE INCLUDED INTO THE PUBLISHED LIBRARY, but I would like to try and make an odometry class so we can drop RR completely.

package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class DThreeWheelOdo implements Odometry {

    public DOdometryPod[] odoPods;

    double x, y, heading, inchesPerTick;

    //First val is trackwith, second value is forward offset, third is auxiliary trackwidth
    double[] dimensions;
    DThread thread = new NullThread();

    ValueStorage vals;

    public DThreeWheelOdo(double x, double y, double heading, ValueStorage vals, DOdometryPod[] odoPods, double inchesPerTick, double... dimensions){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.odoPods = odoPods;
        this.vals = vals;
        this.dimensions = dimensions;
        this.inchesPerTick = inchesPerTick;
    }

    public double[] podPositions() {
        return new double[]{odoPods[0].get()[0], odoPods[1].get()[0], odoPods[2].get()[0]};
    }

    public void set(double value) {
        //Do nothing
    }

    public int getPartNum() {
        //Return -1
        return -1;
    }

    public double[] get() {
        return new double[]{x, y, heading};
    }

    public void setHardware(double value) {
        //Do nothing
    }

    public double[] getHardware() {
        //Do nothing
        return new double[0];
    }

    public void endThreads() {
        if(thread.isAlive()) thread.Stop();
    }

    public Point getPosition() {
        return new Point(x, y, heading);
    }

    public void setPosition(Point newPos) {
        x = newPos.x;
        y = newPos.y;
        heading = newPos.heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public void beginTracking() {
        thread = new OdometryThread(this, vals, dimensions, x, y, heading, inchesPerTick);
        thread.start();
    }

    public void endTracking() {
        thread.Stop();
    }
}
*/