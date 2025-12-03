package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BLUE - Observation Zone Auto", group = "DECODE")
public class BlueObservationZoneAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Starting pose - Blue Observation Zone (bottom right)
        Pose2d startPose = new Pose2d(48, -48, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("BLUE Observation Zone Autonomous Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ===== PHASE 1: SCORE 3 PRELOADED ARTIFACTS =====
        telemetry.addData("Phase", "Scoring preloaded artifacts");
        telemetry.update();

        // Move to blue observation zone scoring position
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(new Vector2d(56, -48), Math.toRadians(270))
                        .build()
        );

        //autoAlign();
        //shootArtifacts(3);
        sleep(500);

        // ===== PHASE 2: COLLECT TOP RIGHT GROUP (farthest - deny opponent!) =====
        telemetry.addData("Phase", "Collecting top right artifacts");
        telemetry.update();

        // Navigate to top right artifact group
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(48, 24), Math.toRadians(180))
                        .build()
        );

        // Start intake and DRIVE THROUGH the 3 balls
        //startIntake();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineTo(new Vector2d(24, 24)) // Drive through all 3 balls
                        .build()
        );
        //stopIntake();
        sleep(300);

        // ===== PHASE 3: RETURN TO OBSERVATION ZONE AND SCORE =====
        telemetry.addData("Phase", "Returning to observation zone");
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(56, -48), Math.toRadians(270))
                        .build()
        );

        // Score collected artifacts
        telemetry.addData("Phase", "Scoring collected artifacts");
        telemetry.update();

        //autoAlign();
        //shootArtifacts(3);
        sleep(500);

        // ===== PHASE 4: PARK IN OBSERVATION ZONE =====
        telemetry.addData("Phase", "Parking");
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(48, -56), Math.toRadians(270))
                        .build()
        );

        telemetry.addData("Status", "Parked - Autonomous Complete!");
        telemetry.addData("Runtime", "%.1f seconds", getRuntime());
        telemetry.update();
    }

    // ========== PLACEHOLDER FUNCTIONS ==========

    private void autoAlign() {
        // PLACEHOLDER
        sleep(300);
    }

    private void shootArtifacts(int count) {
        // PLACEHOLDER
    }

    private void startIntake() {
        // PLACEHOLDER - turn on intake motors
    }

    private void stopIntake() {
        // PLACEHOLDER - turn off intake motors
    }
}

/*
 * BLUE Alliance - Observation Zone (Bottom Right)
 * - Starts: Bottom right corner
 * - Scores: Blue observation zone (bottom right area)
 * - Strategy: Collect TOP RIGHT artifacts (farthest - denies opponent)
 * - Drive THROUGH the 3 balls to collect them all at once
 */