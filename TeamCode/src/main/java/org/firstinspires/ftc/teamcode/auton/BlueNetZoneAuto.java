//package org.firstinspires.ftc.teamcode.auton;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "BLUE - Net Zone Auto", group = "DECODE")
//public class BlueNetZoneAuto extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Starting pose - Blue Net Zone (bottom left)
//        Pose2d startPose = new Pose2d(-48, -48, Math.toRadians(0));
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
//
//        telemetry.addLine("BLUE Net Zone Autonomous Ready");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        // ===== PHASE 1: SCORE 3 PRELOADED ARTIFACTS =====
//        telemetry.addData("Phase", "Scoring preloaded artifacts");
//        telemetry.update();
//
//        // Move to blue net zone scoring position
//        Actions.runBlocking(
//                drive.actionBuilder(startPose)
//                        .strafeToLinearHeading(new Vector2d(-56, -48), Math.toRadians(90))
//                        .build()
//        );
//
//        //autoAlign();
//        //shootArtifacts(3);
//        sleep(500);
//
//        // ===== PHASE 2: COLLECT BOTTOM LEFT GROUP (drive through 3 balls) =====
//        telemetry.addData("Phase", "Collecting bottom left artifacts");
//        telemetry.update();
//
//        // Position before artifact group
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-48, -24), Math.toRadians(0))
//                        .build()
//        );
//
//        // Start intake and DRIVE THROUGH the 3 balls
//        //startIntake();
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .lineTo(new Vector2d(-24, -24)) // Drive through all 3 balls
//                        .build()
//        );
//        //stopIntake();
//        sleep(300);
//
//        // Return to net zone
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-56, -48), Math.toRadians(90))
//                        .build()
//        );
//        //autoAlign();
//        //shootArtifacts(3);
//        sleep(500);
//
//        // ===== PHASE 3: COLLECT BOTTOM MIDDLE GROUP =====
//        telemetry.addData("Phase", "Collecting bottom middle artifacts");
//        telemetry.update();
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(0))
//                        .build()
//        );
//
//        // Drive through the 3 balls
//        //startIntake();
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .lineTo(new Vector2d(0, -24)) // Drive through middle group
//                        .build()
//        );
//        //stopIntake();
//        sleep(300);
//
//        // Return and score
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-56, -48), Math.toRadians(90))
//                        .build()
//        );
//        //autoAlign();
//        //shootArtifacts(3);
//        sleep(500);
//
//        // ===== PHASE 4: COLLECT BOTTOM RIGHT GROUP (if time allows) =====
//        if (getRuntime() < 25.0) {
//            telemetry.addData("Phase", "Collecting bottom right artifacts");
//            telemetry.update();
//
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(new Vector2d(24, -24), Math.toRadians(0))
//                            .build()
//            );
//
//            // Drive through the 3 balls
//            //startIntake();
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .lineTo(new Vector2d(48, -24)) // Drive through right group
//                            .build()
//            );
//            //stopIntake();
//            sleep(300);
//
//            // Final scoring run
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(new Vector2d(-56, -48), Math.toRadians(90))
//                            .build()
//            );
//            //autoAlign();
//            //shootArtifacts(3);
//        }
//
//        telemetry.addData("Status", "Autonomous Complete!");
//        telemetry.addData("Runtime", "%.1f seconds", getRuntime());
//        telemetry.update();
//    }
//
//    // ========== PLACEHOLDER FUNCTIONS ==========
//
//    private void autoAlign() {
//        // PLACEHOLDER
//        sleep(300);
//    }
//
//    private void shootArtifacts(int count) {
//        // PLACEHOLDER
//    }
//
//    private void startIntake() {
//        // PLACEHOLDER - turn on intake motors
//    }
//
//    private void stopIntake() {
//        // PLACEHOLDER - turn off intake motors
//    }
//}
//
///*
// * BLUE Alliance - Net Zone (Bottom Left)
// * - Starts: Bottom left corner
// * - Scores: Blue net zone (bottom left area)
// * - Strategy: Drive THROUGH artifact groups to collect all 3 at once
// * - Collects: Bottom left → middle → right groups
// */