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

@Autonomous(name="Auto: 6-Tile Path", group="TestRobot")
public class TestingAutonomous extends OpMode {

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
                    follower.followPath(driveToPark1);
                    pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(moveForward);
                    pathState = 2;
                }
                break;
            case 2:
                // Segment 3 is done. Start the timer and move to the sweep state.
                if (!follower.isBusy()) {
                    actionTimer.reset(); // Start the 5-second countdown
                    pathState = 3;
                }
                break;
            case 3:
                updateServoSweep();

                // Once 5 seconds have passed, start the next path
                if (actionTimer.seconds() > 5.0) {
                    follower.followPath(strafeToB2);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(driveToB1);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(parkAtStart);
                    pathState = 6;
                }
                break;
            case 6:
                // Segment 6 is done. Start the timer for the final sweep.
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = 7;
                }
                break;
            case 7:
                updateServoSweep();

                if (actionTimer.seconds() > 5.0) {
                    pathState = 8;
                }
                break;
            case 8:
                // Everything is 100% finished
                telemetry.addData("Status", "Full Sequence & Sweeps Complete!");
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
