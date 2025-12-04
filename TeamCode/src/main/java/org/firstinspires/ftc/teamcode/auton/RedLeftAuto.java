package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "RedLeftAuto_Bot1Path_DriveOnly", group = "Autonomous")
public class RedLeftAuto extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set initial pose matching botTag1 start
        Pose2d startPose = new Pose2d(-36, 36, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Hardware initialized, waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build the botTag1 trajectory (drive only)
        Action bot1DriveAction = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12, 32), 0)
                .strafeToConstantHeading(new Vector2d(-12, 55))
                .strafeToConstantHeading(new Vector2d(-12, 46))
                .splineToConstantHeading(new Vector2d(0, 12), 0)
                .splineToConstantHeading(new Vector2d(12, 32), 0)
                .strafeToConstantHeading(new Vector2d(12, 55))
                .strafeToConstantHeading(new Vector2d(12, 46))
                .splineToConstantHeading(new Vector2d(0.01, 12), 0)
                .splineToConstantHeading(new Vector2d(36, 32), 0)
                .strafeToConstantHeading(new Vector2d(36, 55))
                .splineToConstantHeading(new Vector2d(-12, 12), 0)
                .strafeToConstantHeading(new Vector2d(-36, 36))
                .build();

        // Execute the trajectory
        Actions.runBlocking(bot1DriveAction);

        telemetry.addLine("Bot1 drive-only path complete.");
        telemetry.update();
    }
}
