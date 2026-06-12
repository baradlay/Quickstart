package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    //This class is holding some methods that the class MecanumWheelsBehavior uses
    private double rateLimit;
    private double currentOutput = 0.0;
    private ElapsedTime timer;

    /**
     * @param rateLimit The maximum allowed change in power per second
     */
    public SlewRateLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
        this.timer = new ElapsedTime();
    }

    /**
     * Calculates the smoothed output based on the target input.
     * @param target The raw joystick input
     * @return The smoothed, rate-limited output (or 0 for instant braking)
     */
    public double calculate(double target) {
        double deltaTime = timer.seconds();
        timer.reset(); // Restart the stopwatch for the next loop

        // --- THE ASYMMETRICAL BYPASS ---
        // If the driver lets go of the stick (target is close to 0), bypass the limit.
        if (Math.abs(target) < 0.05) {
            currentOutput = 0.0;
            return currentOutput;
        }

        // --- STANDARD LIMITER MATH ---
        double maxChange = rateLimit * deltaTime;
        double error = target - currentOutput;

        if (Math.abs(error) <= maxChange) {
            currentOutput = target;
        } else {
            currentOutput += Math.signum(error) * maxChange;
        }

        return currentOutput;
    }
}