package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Left Encoder Auto", group="Blue")
public class BlueLeftEncoderAuto extends AutonConfig {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        telemetry.addLine("Blue Left Encoder Auto Initialized");
        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            driveBackward(63, DRIVE_POWER);
            scoreArtifacts();
            driveSpline(18, -18, 45, SplinePower);
            intakeArtifacts();
            driveForward(30, DRIVE_POWER);
            stopIntake();

            driveSpline(-33, 12, -45, SplinePower);
            scoreArtifacts();

            driveSpline(24, -26, 45, SplinePower);
            intakeArtifacts();
            driveForward(32, DRIVE_POWER);
            stopIntake();

            strafeLeft(12, STRAFE_POWER);
            driveSpline(-54, 39, -45, SplinePower);
            scoreArtifacts();

            driveSpline(41, -14, -135, SplinePower);

            telemetry.addLine("zone parked");
            telemetry.update();
        }
    }
}