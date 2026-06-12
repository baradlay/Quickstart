package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto: Complex 6-Tile Path", group="Team 25919")
public class TestingAutonomous extends OpMode {

    private Follower follower;
    private PathChain pushGamePiece, driveToPark1, moveForward, strafeToB2, driveToB1, parkAtStart;
    private int pathState;

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

        // ADD THIS: Initializes the Panels Field with Pedro Pathing offsets
        Drawing.init();

        // Continuously broadcast the starting position to Panels!
        Drawing.drawDebug(follower);

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
        follower.followPath(pushGamePiece);
    }

    @Override
    public void loop() {
        follower.update();

        // ADD THIS: Initializes the Panels Field with Pedro Pathing offsets
        Drawing.init();

        // Expand the state machine to run all 6 segments back-to-back
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) { follower.followPath(driveToPark1); pathState = 1; }
                break;
            case 1:
                if (!follower.isBusy()) { follower.followPath(moveForward); pathState = 2; }
                break;
            case 2:
                if (!follower.isBusy()) { follower.followPath(strafeToB2); pathState = 3; }
                break;
            case 3:
                if (!follower.isBusy()) { follower.followPath(driveToB1); pathState = 4; }
                break;
            case 4:
                if (!follower.isBusy()) { follower.followPath(parkAtStart); pathState = 5; }
                break;
            case 5:
                if (!follower.isBusy()) {
                    telemetry.addData("Status", "Full Sequence Complete!");
                }
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
