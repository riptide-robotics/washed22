package org.firstinspires.ftc.teamcode.AUTONOMOUS;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.PIDController;

@Autonomous(name="  meet Regional woah", group="Cool Stuffs")

public class REGIONAL extends LinearOpMode {


    //PID STUFF Please tune this stuff
    public static double kProportional = 0;
    public static double kIntegral = 0;
    public static double kDerivative = 0;

    //PID FeedForward
    public static double kVelocity = 0;
    public static double kAcceleration = 0;
    public static double kStatic = 0;

    //beginning pos
    public static double startpos = 0;





    /*
    use these to set position and speed and stuff
    controller.setTargetPosition(position);
    controller.setTargetVelocity(velocity);
    controller.setTargetAcceleration(acceleration);
     */


    @Override
    public void runOpMode()
    {
        PIDCoefficients coeffs = new PIDCoefficients(kProportional, kIntegral, kDerivative);

        // Voltage neccessarry to like do error stuff
        PIDFController controller = new PIDFController(coeffs, kVelocity, kAcceleration, kStatic);

        controller.setTargetPosition(startpos);


        /*
        Preload cone

        Move Elbow out, scan April tag, move back in


        Road runner
        Move to 3rd tile, turn a certain degree
        extend horizontal and start cycling, Cap preloaded cone
            Contour check loop.
        Evaluate if we have enough time
        If we barley do then we go to parking spot
         */

    }
}
