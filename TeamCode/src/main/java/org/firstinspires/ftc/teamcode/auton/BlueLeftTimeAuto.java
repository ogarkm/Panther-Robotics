package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TransferSys;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


//THIS IS HARDCODED TIME BASED AUTON IN CASE WE CANT GET OUR ROADRUNNER DONE.
//THIS WILL BE UNRELIABLE AS THIS CAN VARY DUE TO BATTERY VOLTAGE, BUT IT MAY WORK ON FULL BATTERY
//AS LONG AS I TUNE IT ON FULL BATTERY (etc)

@Autonomous(name="Blue Left Time Auto", group="Blue")
public class BlueLeftTimeAuto extends LinearOpMode {

    private hwMap.DriveHwMap driveTrain;
    private hwMap.IntakeHwMap intake;
    private hwMap.TransferHwMap transfer;
    private hwMap.TurretHwMap turret;

    // Power levels
    private static final double DRIVE_POWER = 0.6;
    private static final double STRAFE_POWER = 0.6;
    private static final double TURN_POWER = 0.5;
    private static final double SLOW_POWER = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        driveTrain = new hwMap.DriveHwMap(hardwareMap);
        intake = new hwMap.IntakeHwMap(hardwareMap);
        transfer = new hwMap.TransferHwMap(hardwareMap);
        turret = new hwMap.TurretHwMap(hardwareMap);

        telemetry.addLine("Blue Left Auto Initialized");
        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // === YOUR AUTONOMOUS SEQUENCE HERE ===

            // Example sequence (adjust times and movements for your robot):

            // 1. Drive forward to scoring position
            driveForward(DRIVE_POWER, 1500); // 1.5 seconds forward

            // 2. Strafe right to align
            strafeRight(STRAFE_POWER, 800); // 0.8 seconds right

            // 3. Score preloaded artifacts
            scoreArtifacts();

            // 4. Drive backward
            driveBackward(DRIVE_POWER, 1000); // 1 second backward

            // 5. Turn to face artifacts
            turnLeft(TURN_POWER, 600); // 0.6 seconds turning

            // 6. Intake artifacts
            intakeArtifacts(2000); // 2 seconds of intaking

            // 7. Turn back to scoring position
            turnRight(TURN_POWER, 600);

            // 8. Drive to scoring position again
            driveForward(DRIVE_POWER, 1000);

            // 9. Score collected artifacts
            scoreArtifacts();

            // 10. Park
            strafeLeft(DRIVE_POWER, 1200);
            driveBackward(SLOW_POWER, 500);

            telemetry.addLine("Autonomous Complete!");
            telemetry.update();
        }
    }

    // ========== DRIVE METHODS ==========

    private void driveForward(double power, long milliseconds) {
        driveTrain.setMotorPowers(power, power, power, power);
        sleep(milliseconds);
        stopDrive();
    }

    private void driveBackward(double power, long milliseconds) {
        driveTrain.setMotorPowers(-power, -power, -power, -power);
        sleep(milliseconds);
        stopDrive();
    }

    private void strafeRight(double power, long milliseconds) {
        // Mecanum strafe: FL=+, FR=-, BL=-, BR=+
        driveTrain.setMotorPowers(power, -power, -power, power);
        sleep(milliseconds);
        stopDrive();
    }

    private void strafeLeft(double power, long milliseconds) {
        // Mecanum strafe: FL=-, FR=+, BL=+, BR=-
        driveTrain.setMotorPowers(-power, power, power, -power);
        sleep(milliseconds);
        stopDrive();
    }

    private void turnRight(double power, long milliseconds) {
        // Turn in place: left motors forward, right motors backward
        driveTrain.setMotorPowers(power, -power, power, -power);
        sleep(milliseconds);
        stopDrive();
    }

    private void turnLeft(double power, long milliseconds) {
        // Turn in place: left motors backward, right motors forward
        driveTrain.setMotorPowers(-power, power, -power, power);
        sleep(milliseconds);
        stopDrive();
    }

    private void stopDrive() {
        driveTrain.stopMotors();
        sleep(100); // Small pause for stability
    }

    // ========== GAME ELEMENT METHODS ==========

    private void intakeArtifacts(long milliseconds) {
        // Run intake motors
        intake.frontIntakeMotor.setPower(1.0);
        intake.backIntakeMotor.setPower(1.0);
        sleep(milliseconds);
        intake.frontIntakeMotor.setPower(0);
        intake.backIntakeMotor.setPower(0);
        sleep(200);

        // Index the artifacts
        for (int i = 1; i <= 3; i++) {
            transfer.detectArtifactColor(i);
        }
    }

    private void scoreArtifacts() {
        // Spin up flywheels
        turret.setTurretPower(0.7);
        sleep(1000); // Wait for flywheels to spin up

        // Flick artifacts one by one
        for (int slot = 1; slot <= 3; slot++) {
            transfer.setTransferPos(slot, true); // Flick up
            sleep(300); // Wait for artifact to shoot
            transfer.setTransferPos(slot, false); // Flick down
            sleep(200); // Small delay between shots
        }

        // Stop flywheels
        turret.turretOff();
        sleep(300);
    }

    // ========== UTILITY METHODS ==========

    // Drive forward with distance feedback (if you want more precision)
    private void driveForwardDistance(double power, int targetTicks) {
        driveTrain.resetEncoders();

        while (opModeIsActive()) {
            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = (Math.abs(positions[0]) + Math.abs(positions[1]) +
                    Math.abs(positions[2]) + Math.abs(positions[3])) / 4;

            if (avgPosition >= targetTicks) {
                break;
            }

            driveTrain.setMotorPowers(power, power, power, power);
            telemetry.addData("Position", avgPosition);
            telemetry.addData("Target", targetTicks);
            telemetry.update();
        }

        stopDrive();
    }

    // Diagonal movements (if needed)
    private void driveDiagonalForwardRight(double power, long milliseconds) {
        // FL and BR move, FR and BL stop
        driveTrain.setMotorPowers(power, 0, 0, power);
        sleep(milliseconds);
        stopDrive();
    }

    private void driveDiagonalForwardLeft(double power, long milliseconds) {
        // FR and BL move, FL and BR stop
        driveTrain.setMotorPowers(0, power, power, 0);
        sleep(milliseconds);
        stopDrive();
    }
}