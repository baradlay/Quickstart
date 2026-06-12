package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MecanunWheelsBehavior", group="TestRobot")
public class MecanumWheelsBehavior extends OpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    // 1. Declare your limiters
    private SlewRateLimiter forwardLimiter;
    private SlewRateLimiter strafeLimiter;
    private SlewRateLimiter turnLimiter;

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

        // DIRECTION RULE: Reverse the left side motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // 2. Initialize the limiters with a max rate of 2.0 units per second
        forwardLimiter = new SlewRateLimiter(0.5);
        strafeLimiter = new SlewRateLimiter(0.5);
        turnLimiter = new SlewRateLimiter(0.5); // You can make turning faster/slower if you want!

        telemetry.addData("Status", "Limiters Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 3. Read the raw joysticks, but immediately pass them through the limiters
        double y = forwardLimiter.calculate(-gamepad1.left_stick_y);
        double x = strafeLimiter.calculate(gamepad1.left_stick_x);
        double rx = turnLimiter.calculate(gamepad1.right_stick_x);

        // 4. Standard Mecanum math using the smoothed values
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx);
        double blPower = (y - x + rx);
        double frPower = (y - x - rx);
        double brPower = (y + x - rx);

        // Apply a 5% speed reduction to the overpowering left side
        flPower = flPower * .88;
        blPower = blPower * .88;

        leftFront.setPower(flPower/ denominator);
        leftRear.setPower(blPower/ denominator);
        rightFront.setPower(frPower/ denominator);
        rightRear.setPower(brPower/ denominator);

        // Telemetry to see the limiters at work
        telemetry.addData("Raw Joystick Y", "%.2f", -gamepad1.left_stick_y);
        telemetry.addData("Smoothed Y Output", "%.2f", y);
        telemetry.update();
    }
}