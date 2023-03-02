package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDController {
    // PID terms
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    // previous error is used for integration (trapezoidal) and for derivation
    public double previousError = 0;
    public double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();    public boolean hasRun = false;
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.seconds();
        timer.reset();
        return dt;
    }
    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        double dt = getDT();
        // intuitive
        double error = target - state;
        // derivative value is calculated as the slope between the current state and the previous state
        // this is fake calculus! burn me at the stake as a heritic
        double derivative = (error - previousError)/dt;
        // fake trapezoidal integration here as well, with dt being the length of the box.
        // Add to the running integralSum.
        integralSum += ((target + state)/2) * dt;
        previousError = error;
        return error * Kp + integralSum * Ki + derivative * Kd;


    }
}