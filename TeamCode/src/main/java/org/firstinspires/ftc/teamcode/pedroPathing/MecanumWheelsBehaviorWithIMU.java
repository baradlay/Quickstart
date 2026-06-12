package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="MecanumWheelsBehaviorWithIMU", group="TestRobot")
public class MecanumWheelsBehaviorWithIMU extends OpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private IMU imu;

    // Slew rate limiters
    private SlewRateLimiter forwardLimiter;
    private SlewRateLimiter strafeLimiter;
    private SlewRateLimiter turnLimiter;

    // Heading hold variables
    private double targetHeading = 0.0;
    private final double Kp = 0.015; // Proportional tuning factor

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        // Use direct raw power mode for driving
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // DIRECTION RULE: Reverse the left side motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize limiters (2.0 allows reaching full power in 0.5 seconds)
        forwardLimiter = new SlewRateLimiter(1.25);
        strafeLimiter = new SlewRateLimiter(1.25);
        turnLimiter = new SlewRateLimiter(1.25);

        // INITIALIZE THE IMU WITH THE CORRECT ORIENTATION
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,      // Logo points to the sky
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT      // USB ports face the left side
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addData("Status", "Hardware Mapped and IMU Configured.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Guard step to protect against null pointers if hardware initialization fails
        if (imu == null) {
            telemetry.addData("Error", "IMU failed to initialize!");
            telemetry.update();
            return;
        }

        // Read and smooth joystick translation vectors
        double y = forwardLimiter.calculate(-gamepad1.left_stick_y);
        double x = strafeLimiter.calculate(gamepad1.left_stick_x);

        double rawTurnInput = gamepad1.right_stick_x;
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double rx;

        // Active Heading Hold Decision Logic
        if (Math.abs(rawTurnInput) > 0.05) {
            // Driver is manual turning
            rx = turnLimiter.calculate(rawTurnInput);
            targetHeading = currentHeading;
        } else {
            // Driver let go of turn stick - activate automated hold
            turnLimiter.calculate(0);
            double error = targetHeading - currentHeading;

            // Normalize error to the shortest angular distance
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;

            // Apply a 1.0 degree deadband to prevent micro-shaking
            if (Math.abs(error) < 1.0) {
                rx = 0.0;
            } else {
                rx = turnLimiter.calculate(error * Kp);
            }
        }

        // Standard Mecanum math combinations
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx);
        double blPower = (y - x + rx);
        double frPower = (y - x - rx);
        double brPower = (y + x - rx);

        // Compensate for weight distribution imbalance on the left side
        flPower = flPower * 0.88;
        blPower = blPower * 0.88;

        // Send normalized power calculations directly to the wheels
        leftFront.setPower(flPower / denominator);
        leftRear.setPower(blPower / denominator);
        rightFront.setPower(frPower / denominator);
        rightRear.setPower(brPower / denominator);

        telemetry.addData("Heading Target vs Actual", "Target: %.1f | Current: %.1f", targetHeading, currentHeading);
        telemetry.addData("Correction Power (rx)", "%.3f", rx);
        telemetry.update();
    }
}