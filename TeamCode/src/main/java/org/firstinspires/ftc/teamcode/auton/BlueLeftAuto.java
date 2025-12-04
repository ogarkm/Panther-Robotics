package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueLeftAuto_Bot2Path_DriveOnly", group = "Autonomous")
public class BlueLeftAuto extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set initial pose matching mirrored Bot2 start
        Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Hardware initialized, waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build the mirrored Bot2 trajectory (drive only)
        Action bot2DriveAction = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12, -32), 0)
                .strafeToConstantHeading(new Vector2d(-12, -55))
                .strafeToConstantHeading(new Vector2d(-12, -46))
                .splineToConstantHeading(new Vector2d(0, -12), 0)
                .splineToConstantHeading(new Vector2d(12, -32), 0)
                .strafeToConstantHeading(new Vector2d(12, -55))
                .strafeToConstantHeading(new Vector2d(12, -46))
                .splineToConstantHeading(new Vector2d(0.01, -12), 0)
                .splineToConstantHeading(new Vector2d(36, -32), 0)
                .strafeToConstantHeading(new Vector2d(36, -55))
                .splineToConstantHeading(new Vector2d(-12, -12), 0)
                .strafeToConstantHeading(new Vector2d(-36, -36))
                .build();

        // Execute the trajectory
        Actions.runBlocking(bot2DriveAction);

        telemetry.addLine("Bot2 drive-only path complete.");
        telemetry.update();
    }
}
