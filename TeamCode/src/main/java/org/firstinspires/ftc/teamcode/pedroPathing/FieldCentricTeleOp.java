package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="FieldCentricTelOp", group="TestRobot")
public class FieldCentricTeleOp extends OpMode {

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

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        forwardLimiter = new SlewRateLimiter(1.25);
        strafeLimiter = new SlewRateLimiter(1.25);
        turnLimiter = new SlewRateLimiter(1.25);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addData("Status", "Field-Centric Hardware Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (imu == null) {
            telemetry.addData("Error", "IMU failed to initialize!");
            telemetry.update();
            return;
        }

        // --- FIELD CENTRIC CALIBRATION RESET ---
        if (gamepad1.start) {
            imu.resetYaw();
            targetHeading = 0.0; // Reset the P-Controller target to match the new zero
        }

        // 1. Read and smooth joystick translation vectors
        double y = forwardLimiter.calculate(-gamepad1.left_stick_y);
        double x = strafeLimiter.calculate(gamepad1.left_stick_x);
        double rawTurnInput = gamepad1.right_stick_x;

        // 2. Read current heading in both Degrees (for the P-Controller) and Radians (for Field-Centric math)
        double currentHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double botHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rx;

        // --- FIELD CENTRIC ROTATION MATH ---
        // Rotate the driver's X/Y inputs by the robot's current heading
        double rotX = x * Math.cos(-botHeadingRad) - y * Math.sin(-botHeadingRad);
        double rotY = x * Math.sin(-botHeadingRad) + y * Math.cos(-botHeadingRad);

        // Apply the 1.1 strafe correction multiplier to the rotated X axis
        rotX = rotX * 1.1;

        // --- HEADING HOLD DECISION LOGIC ---
        if (Math.abs(rawTurnInput) > 0.05) {
            // Driver is manually turning
            rx = turnLimiter.calculate(rawTurnInput);
            targetHeading = currentHeadingDeg;
        } else {
            // Automated heading hold
            turnLimiter.calculate(0);
            double error = targetHeading - currentHeadingDeg;

            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;

            if (Math.abs(error) < 1.0) {
                rx = 0.0;
            } else {
                rx = turnLimiter.calculate(error * Kp);
            }
        }

        // --- FINAL MECANUM MATH ---
        // Notice we are using rotX and rotY here instead of the raw x and y
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx);
        double blPower = (rotY - rotX + rx);
        double frPower = (rotY - rotX - rx);
        double brPower = (rotY + rotX - rx);

        // Compensate for weight distribution imbalance on the left side
        flPower = flPower * 0.88;
        blPower = blPower * 0.88;

        // Send power directly to the wheels
        leftFront.setPower(flPower / denominator);
        leftRear.setPower(blPower / denominator);
        rightFront.setPower(frPower / denominator);
        rightRear.setPower(brPower / denominator);

        // Telemetry for the driver
        telemetry.addData("Field-Centric Mode", "Active (Press START to Zero)");
        telemetry.addData("Target Heading", "%.1f", targetHeading);
        telemetry.addData("Current Heading", "%.1f", currentHeadingDeg);
        telemetry.update();
    }
}
