package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Debug: Raw OTOS Test", group="Test")
public class RawOtosTest extends OpMode {

    private SparkFunOTOS myOtos;

    @Override
    public void init() {
        // Grab the sensor directly from the hardware map
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // Configure standard units
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // X is -4 inches (back), Y is 0 (centered), Heading is 90 degrees rotated.
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-4, 0, 90);
        myOtos.setOffset(offset);

        // Force a hard calibration and reset
        myOtos.calibrateImu();
        myOtos.resetTracking();

        telemetry.addData("Status", "Initialized. Push robot forward 48 inches.");
    }

    @Override
    public void loop() {
        // Read the exact raw data
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        telemetry.addData("Raw X (Inches)", pos.x);
        telemetry.addData("Raw Y (Inches)", pos.y);
        telemetry.addData("Raw Heading", pos.h);
        telemetry.update();
    }
}