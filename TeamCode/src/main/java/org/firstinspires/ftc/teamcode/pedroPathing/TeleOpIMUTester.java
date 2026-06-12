package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name="IMUTester", group="TestRobot")
public class TeleOpIMUTester extends OpMode {
    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        // 1. Define the orientation parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,        // The REV logo points toward the sky
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT     // The USB ports point toward the front of the robot
        ));

// 2. Feed those parameters into the IMU hardware instance
        imu.initialize(parameters);

    }

    @Override
    public void loop() {
        // A quick check makes Android Studio happy and prevents crashes if init failed
        if (imu == null) {
            telemetry.addData("Error", "IMU not initialized!");
            telemetry.update();
            return; // Exit the loop early safely
        }

        // Android Studio now sees the check above and removes the warning!
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("RAW IMU YAW", currentHeading);
        telemetry.update();
    }
}
