package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="no horiz movement", group="FINAL TELE")
public class noHorizontalMovement extends LinearOpMode {
    ElapsedTime clawPos = new ElapsedTime();

    public enum lifestate{
        OPEN,
        CLOSED
    }
    lifestate state = lifestate.OPEN;

    @Override
    public void runOpMode() throws InterruptedException {
        // clawPos.reset();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");
        DcMotor leftHoriz = hardwareMap.dcMotor.get("leftHoriz");
        DcMotor rightHoriz = hardwareMap.dcMotor.get("rightHoriz");
        DcMotor leftVert = hardwareMap.dcMotor.get("leftVert");
        DcMotor rightVert = hardwareMap.dcMotor.get("rightVert");
        //run all motors using the encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHoriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo servo0 = hardwareMap.servo.get("servo0out");
        Servo servo1 = hardwareMap.servo.get("servo1out");
        Servo servo2 = hardwareMap.servo.get("servo2arm");
        Servo servo3 = hardwareMap.servo.get("servo3arm");
        Servo servo4 = hardwareMap.servo.get("servo4wrist");
        Servo servo5 = hardwareMap.servo.get("servo5wrist");
        Servo servo6 = hardwareMap.servo.get("servo6claw");


        //DcMotor slides = hardwareMap.dcMotor.get("slides");
        //Servo servo0 = hardwareMap.servo.get("servo0");
        //Servo servo1 = hardwareMap.servo.get("servo1");
        //slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double offset = 0;
        boolean bool = true;
        double v = 0.85;


        // Reverse thv e right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            double y = 0.55 * gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -0.55 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -0.55 * gamepad1.right_stick_x;
            if(gamepad1.x)
            {
                y = 1.77 * y;
                x = 1.77 * x;
                rx = 1.77 * rx;
            }
            if(gamepad1.y)
            {
                y = 0.66 * y;
                x = 0.66 * x;
                rx = 0.66 * rx;
            }




           /*if(gamepad2.a)
           {
               bool != bool;
           }

           if(bool)
           {
               servo0.setPosition(0.2);
               servo1.setPosition(0.8);
           }
           else
           {
               servo0.setPosition(1);
               servo1.setPosition(0);
           }*/

            double botHeading = -imu.getAngularOrientation().firstAngle;
            // Read inverse IMU heading, as the IMU heading is CW positive
            if(gamepad1.a)
            {
                offset = -imu.getAngularOrientation().firstAngle;
            }
            if(gamepad1.b)
            {
                offset = 0;
            }
            double rotX = x * Math.cos(botHeading - offset) - y * Math.sin(botHeading - offset);
            double rotY = x * Math.sin(botHeading - offset) + y * Math.cos(botHeading - offset);
            if(gamepad2.a)
            {
                servo2.setPosition(0.18);
                servo3.setPosition(0.82);
            }
            if(gamepad2.left_trigger > 0.2)
            {
                servo3.setPosition(0.4);
                servo2.setPosition(0.6);
            }
            if(gamepad2.b)
            {
                servo2.setPosition(0.4);
                servo3.setPosition(0.6);
            }
            if(gamepad2.right_trigger > 0.2)
            {
                servo2.setPosition(0.26);
                servo3.setPosition(0.74);
            }
            if(gamepad2.x)
            {
                servo6.setPosition(1);
            }
            if(gamepad2.y)
            {
                servo6.setPosition(0.55);
            }
            if(gamepad2.dpad_up)
            {
                leftVert.setPower(-0.85);
                rightVert.setPower(0.85);
            }
            else if(gamepad2.dpad_down)
            {
                leftVert.setPower(0.85);
                rightVert.setPower(-0.85);
            }
            else
            {
                leftVert.setPower(-0.01);
                rightVert.setPower(0.01);
            }

            if(gamepad2.left_bumper)
            {
                servo4.setPosition(0.3);
                servo5.setPosition(0.7);
            }
            else if(gamepad2.right_bumper)
            {
                servo4.setPosition(0.7);
                servo5.setPosition(0.3);
            }
            else {
                servo4.setPosition(0.5);
                servo5.setPosition(0.5);
            }
            if(gamepad2.dpad_left)
            {
                servo0.setPosition(0.95);
                servo1.setPosition(0.05);
            }
            if(gamepad2.dpad_right)
            {
                servo0.setPosition(0.25);
                servo1.setPosition(0.75);
            }
            if(gamepad1.dpad_up)
            {
                servo0.setPosition(0.75);
                servo1.setPosition(0.25);
            }





            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least onex is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            telemetry.addData("y speed:", y);
            telemetry.addData("x speed:", x);
            telemetry.addData("right Vert", rightVert.getCurrentPosition());
            telemetry.addData("left Vert", leftVert.getCurrentPosition());

            telemetry.update();
        }
    }
}


