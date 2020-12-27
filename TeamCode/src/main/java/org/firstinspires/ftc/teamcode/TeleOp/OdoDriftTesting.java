package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Config
public class OdoDriftTesting extends LinearOpMode {

    public static int leftFrontTarget = 5000, leftBackTarget = 5000, rightBackTarget = 5000, rightFrontTarget = 5000;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");

        DcMotor frontEncoder = hardwareMap.get(DcMotor.class, "frontEncoder");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setTargetPosition(leftFrontTarget);
        leftRear.setTargetPosition(leftBackTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        rightRear.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(0.5);
        leftRear.setPower(0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(0.5);

        while(!isStopRequested()) {
            telemetry.addData("Front encoder position: ", frontEncoder.getCurrentPosition());
            telemetry.addData("Left front: ", leftFront.getCurrentPosition());
            telemetry.addData("Left back: ", leftRear.getCurrentPosition());
            telemetry.addData("Right back: ", rightRear.getCurrentPosition());
            telemetry.addData("Right front: ", rightFront.getCurrentPosition());
            telemetry.update();
        }
    }
}
