package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PidTesting extends LinearOpMode {
    ElapsedTime clawPos = new ElapsedTime();

    public enum lifestate{
        OPEN,
        CLOSED
    }
    lifestate state = lifestate.OPEN;

    @Override
    public void runOpMode() throws InterruptedException {
        clawPos.reset();
        // Declare our motors
        // Make sure your ID's match your configuration
       /*DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
       DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
       DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
       DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");*/
        DcMotor slides = hardwareMap.dcMotor.get("slides");
       /*Servo servo0 = hardwareMap.servo.get("servo0");
       Servo servo1 = hardwareMap.servo.get("servo1");
       slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);/*/
        double offset = 0;
        boolean bool = true;

        // Reverse thv e right side motors
        // Reverse left motors if you are using NeveRests
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        //BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        //imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;



        int kp = 1;
        int ki = 0;
        int kd = 0;

        int reference = 500;
        ElapsedTime timer = new ElapsedTime();
        int integeralSum = 0;
        int lasterror = 0;
        while (opModeIsActive()) {
            if (gamepad2.x) {
                while (slides.getCurrentPosition() <= 500) {
                    int encoderPos = slides.getCurrentPosition();
                    int error = reference - encoderPos;
                    double derivative = (error - lasterror) / timer.seconds();
                    integeralSum += (error * timer.seconds());
                    double out = (kp * error) + (ki * integeralSum) + (kd * derivative);
                    slides.setPower(out);
                    lasterror = error;
                    timer.reset();
                }
                slides.setPower(0.95);
            } else {
                slides.setPower(0);
            }
        }
        //sorry my bad2


        // Read inverse IMU heading, as the IMU heading is CW positive

        telemetry.addData("left", gamepad1.left_bumper);
        telemetry.addData("right1", gamepad1.right_bumper);
        telemetry.update();
    }
}


