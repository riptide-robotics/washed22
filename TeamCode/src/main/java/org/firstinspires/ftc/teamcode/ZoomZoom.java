package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.*;

// blah blah blah


@TeleOp(name="movement teleop", group="Linear Opmode")
// decorator so the robot knows what mode to operate in
@Config
public class ZoomZoom extends LinearOpMode {
    // Declare teleOp members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double positionTracker = 0;

    @Override
    // related to inheritance: runOpMode is a necessary function as you NEED to override the runOpMode in the superclass
    public void runOpMode() {
        //System.out on the phones = telemetry
        Servo servo0 = hardwareMap.servo.get("servo0");
        Servo servo1 = hardwareMap.servo.get("servo1");
        // int encoderValue = 0;
        boolean bool = false;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        //motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //  slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // slides.setPower(0.5);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 0.47; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1 * 0.47; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 0.47;

            servo0.setPosition(positionTracker);
            servo1.setPosition(1 - positionTracker);
            // ReLEASE slide to drop claw.







            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // to move left --


//            if(frontLeftPower == 0 && backLeftPower == 0 && frontRightPower == 0 && backRightPower == 0)
//            {
//                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motorFrontLeft.setTargetPosition(0);
//                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motorFrontRight.setTargetPosition(0);
//                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                motorBackLeft.setTargetPosition(0);
//                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }




            // Show the elapsed game time and wheel power
            telemetry.addData("Position Tracker", positionTracker);



            telemetry.update();
        }
    }
}