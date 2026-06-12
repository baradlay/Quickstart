package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto: 6-Tile Path Multitasking", group="TestRobot")
public class TestingAutonomousMultitasking extends OpMode {

    private Follower follower;
    private PathChain pushGamePiece, driveToPark1, moveForward, strafeToB2, driveToB1, parkAtStart;
    private int pathState;
    private Servo myServo;
    private final ElapsedTime actionTimer = new ElapsedTime();


    // The 7 explicit Poses required for your sequence
    private final Pose startPose = new Pose(9, 36, Math.toRadians(0));
    private final Pose piecePose = new Pose(36, 36, Math.toRadians(0));
    private final Pose park1Pose = new Pose(60, 12, Math.toRadians(90));
    private final Pose forwardPose = new Pose(60, 30, Math.toRadians(90));
    private final Pose b2Pose = new Pose(36, 36, Math.toRadians(90));
    private final Pose b1Pose = new Pose(36, 12, Math.toRadians(270));
    // 360 degrees (2 * PI) is mathematically identical to 0, but ensures a smooth forward rotation!
    private final Pose endPose = new Pose(9, 36, Math.toRadians(360));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Map your servo (make sure the name matches your Driver Hub config!)
        myServo = Constants.createMyServo(hardwareMap);

        // Initializes the Panels Field with Pedro Pathing offsets
        Drawing.init();

        // Segment 1: Start -> Piece
        pushGamePiece = follower.pathBuilder()
                .addPath(new BezierLine(startPose, piecePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // Segment 2: Piece -> C1 Park
        driveToPark1 = follower.pathBuilder()
                .addPath(new BezierCurve(piecePose, new Pose(36, 12, 0), park1Pose))
                .setLinearHeadingInterpolation(piecePose.getHeading(), park1Pose.getHeading())
                .build();

        // Segment 3: Move Forward 1.5ft
        moveForward = follower.pathBuilder()
                .addPath(new BezierLine(park1Pose, forwardPose))
                .setConstantHeadingInterpolation(park1Pose.getHeading())
                .build();

        // Segment 4: Strafe Left to B2
        strafeToB2 = follower.pathBuilder()
                .addPath(new BezierLine(forwardPose, b2Pose))
                .setConstantHeadingInterpolation(forwardPose.getHeading())
                .build();

        // Segment 5: 180 Turn to B1
        driveToB1 = follower.pathBuilder()
                .addPath(new BezierLine(b2Pose, b1Pose))
                .setLinearHeadingInterpolation(b2Pose.getHeading(), b1Pose.getHeading())
                .build();

        // Segment 6: Curve Park at Start
        parkAtStart = follower.pathBuilder()
                .addPath(new BezierCurve(b1Pose, new Pose(16, 12, 0), endPose))
                .setLinearHeadingInterpolation(b1Pose.getHeading(), endPose.getHeading())
                .build();

        telemetry.addData("Status", "Complex Path Generated.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Keep the speed capped so the bot doesn't violently whip around during the 180 spin
        follower.setMaxPower(0.45);
        pathState = 0;
        actionTimer.reset();
        follower.followPath(pushGamePiece);
    }

    @Override
    public void loop() {
        follower.update();

        // Panels interaction
        Drawing.drawDebug(follower);

        // State machine
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    // Reset the timer right before Seg 2 starts so the servo starts at 0
                    actionTimer.reset(); 
                    follower.followPath(driveToPark1);
                    pathState = 1;
                }
                break;

            case 1:
                // --- SWEEPING DURING SEGMENT 2 ---
                updateServoSweep(); 

                if (!follower.isBusy()) {
                    // Optional: Lock the servo back to the center position when the sweep stops?
                    myServo.setPosition(0.25); 
                    follower.followPath(moveForward);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    // Reset the timer again before Seg 2 starts so the servo starts at 0
                    actionTimer.reset(); 
                    follower.followPath(strafeToB2);
                    pathState = 3;
                }
                break;

            case 3:
                // --- SWEEPING DURING SEGMENT 4 ---
                updateServoSweep();

                if (!follower.isBusy()) {
                    // Do NOT reset the timer here so the servo smoothly continues!
                    follower.followPath(driveToB1);
                    pathState = 4;
                }
                break;

            case 4:
                // --- SWEEPING CONTINUES DURING SEGMENT 5 ---
                updateServoSweep();

                if (!follower.isBusy()) {
                    myServo.setPosition(0.25); // Lock it back to center
                    follower.followPath(parkAtStart);
                    pathState = 5;
                }
                break;

            case 5:
                // The sweep is stopped for Segment 6!
                if (!follower.isBusy()) {
                    pathState = 6;
                }
                break;

            case 6:
                // Everything is 100% finished
                telemetry.addData("Status", "Driving & Waving Complete!");
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Timer", actionTimer.seconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
    private void updateServoSweep() {

        double position = 0.25 + (0.25 * Math.sin(actionTimer.seconds() * Math.PI * 2));
        myServo.setPosition(position);
    }


}
