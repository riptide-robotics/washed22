package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class horizPID extends LinearOpMode {

    public static PIDController controller;
    public static double ki = 0.001;
    public static double kp = 0.009;
    public static double kd = 0;

    public static double kf = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 145.1/360.0;


    public void PIDloop(double targetPosition, DcMotor motor1, DcMotor motor2) {
        double integeralSum = 0;
        double lasterror = 0;
        ElapsedTime timer = new ElapsedTime();
        while (motor1.getCurrentPosition() <= 500) {
            double encoderPos = motor1.getCurrentPosition();
            double error = targetPosition - encoderPos;
            double derivative = (error - lasterror) / timer.seconds();
            integeralSum += (error * timer.seconds());
            double out = (kp * error) + (ki * integeralSum) + (kd * derivative);
            motor1.setPower(out);
            motor2.setPower(out);
            telemetry.addData("encoder position:", motor1.getCurrentPosition());
            telemetry.addData("output power:", out);
            telemetry.addData("x?", gamepad2.x);
            telemetry.update();
            lasterror = error;
            timer.reset();
        }
    }

    // helper functions
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(kp, ki, kd);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        DcMotorEx rightHoriz = hardwareMap.get(DcMotorEx.class, "rightHoriz");
        DcMotorEx leftHoriz = hardwareMap.get(DcMotorEx.class, "leftHoriz");
        leftHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHoriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHoriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHoriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Declare our motors
        // Make sure your ID's match your configuration
       /*DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
       DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
       DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
       DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");*/

        /*Servo servo0 = hardwareMap.servo.get("servo0");
       Servo servo1 = hardwareMap.servo.get("servo1");
       slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);/*/
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

        while (opModeIsActive()) {

            //actual pid code?
            int slidePos = rightHoriz.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);
            //useless additional feedforward controller! yay!
            // this is meant for arms to counteract gravity
            double ff = kf;
            double power = pid + ff;
            leftHoriz.setPower(power);
            rightHoriz.setPower(power);
            dashboardTelemetry.addData("slidePos, " , slidePos);
            dashboardTelemetry.addData("target, ", target);
            dashboardTelemetry.addData("power", power);
            dashboardTelemetry.addData("leftSlidePos", -leftHoriz.getCurrentPosition());
            dashboardTelemetry.update();
        }
        //sorry my bad2


        // Read inverse IMU heading, as the IMU heading is CW positive
    }
}
