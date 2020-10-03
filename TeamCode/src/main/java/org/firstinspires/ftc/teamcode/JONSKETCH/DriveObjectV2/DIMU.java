package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DIMU implements DriveObject {

    private BNO055IMU imu;
    private ValueStorage vals;

    private DOThread thread = new NullThread();

    private int partNum;

    public DIMU(ValueStorage vals, HardwareMap hwMap, int partnum) {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; //Default radians
        imu.initialize(parameters);

        this.vals = vals;
        this.partNum = partnum;
    }

    public void set(double value) {
        //Do nothing
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public void setHardware(double value) {
        //Do nothing
    }

    public double[] getHardware() {
        return new double[]{imu.getAngularOrientation().firstAngle, imu.getAngularOrientation().secondAngle, imu.getAngularOrientation().thirdAngle};
    }

    public void endThreads() {
        thread.Stop();
    }

    public void setUnit(BNO055IMU.AngleUnit unit) {
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = unit;
        imu.initialize(param);
    }
}
