package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hware.hwMap;


//THIS IS HARDCODED TIME BASED AUTON IN CASE WE CANT GET OUR ROADRUNNER DONE.
//THIS WILL BE UNRELIABLE AS THIS CAN VARY DUE TO BATTERY VOLTAGE, BUT IT MAY WORK ON FULL BATTERY
//AS LONG AS I TUNE IT ON FULL BATTERY (etc)

@Autonomous(name="Red Net Time Auto", group="Blue")
public class RedNetTimeAuto extends LinearOpMode {

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

        transfer.resetAllFlickers();
        waitForStart();


        if (opModeIsActive()) {
            /*

            //-----------------------------------------------------------------
            // 0. Start turret wheel early so it’s already spinning when you shoot
            //-----------------------------------------------------------------
            turret.setTurretPower(0.70);   // spin flywheel
            sleep(800);                    // brief warm-up

            //-----------------------------------------------------------------
            // 1. Move back a few inches (tune time)
            //-----------------------------------------------------------------
            driveBackward(0.45, 450);      // ~4–6 inches depending on robot

            //-----------------------------------------------------------------
            // 2. Shoot 3 preloads
            //-----------------------------------------------------------------
            for (int slot = 1; slot <= 3; slot++) {
                transfer.setTransferPos(slot, true);
                sleep(300);
                transfer.setTransferPos(slot, false);
                sleep(150);
            }

            //-----------------------------------------------------------------
            // 3. Turn ~30–45° left
            //-----------------------------------------------------------------
            turnLeft(0.50, 400);           // adjust time for your robot

            //-----------------------------------------------------------------
            // 4. Strafe left into intake lane
            //-----------------------------------------------------------------
            strafeLeft(0.55, 500);

            //-----------------------------------------------------------------
            // 5. Drive forward while intaking
            //-----------------------------------------------------------------
            intake.frontIntakeMotor.setPower(1);
            intake.backIntakeMotor.setPower(1);
            driveForward(0.45, 900);       // ~1–1.5 tiles forward
            sleep(100);
            driveBackward(0.45, 200);      // small settle-back
            intake.frontIntakeMotor.setPower(0);
            intake.backIntakeMotor.setPower(0);

            //-----------------------------------------------------------------
            // 6. Drive back to shooting position
            //-----------------------------------------------------------------
            driveBackward(0.55, 800);

            // turn back to original heading for shooting
            turnRight(0.50, 400);

            //-----------------------------------------------------------------
            // 7. Shoot again (assuming 3 artifacts picked up)
            //-----------------------------------------------------------------
            for (int slot = 1; slot <= 3; slot++) {
                transfer.setTransferPos(slot, true);
                sleep(300);
                transfer.setTransferPos(slot, false);
                sleep(150);
            }

            //-----------------------------------------------------------------
            // 8. Repeat cycle to reach 12–15 artifacts if time permits
            //-----------------------------------------------------------------
            // second intake cycle
            turnLeft(0.50, 400);
            strafeLeft(0.55, 500);

            intake.frontIntakeMotor.setPower(1);
            intake.backIntakeMotor.setPower(1);

            driveForward(0.45, 900);   // intake pass 2
            sleep(100);
            driveBackward(0.45, 200);

            intake.frontIntakeMotor.setPower(0);
            intake.backIntakeMotor.setPower(0);

            // return again to shoot
            driveBackward(0.55, 800);
            turnRight(0.50, 400);

            // shoot again (9 artifacts total)
            for (int slot = 1; slot <= 3; slot++) {
                transfer.setTransferPos(slot, true);
                sleep(300);
                transfer.setTransferPos(slot, false);
                sleep(150);
            }

            //-----------------------------------------------------------------
            // 9. Third cycle if there is time left → try to reach 12–15
            //-----------------------------------------------------------------
            // COMMENT OUT if your auton runs out of time.

            // turnLeft(0.50, 400);
            // strafeLeft(0.55, 500);

            // intake.frontIntakeMotor.setPower(1);
            // intake.backIntakeMotor.setPower(1);

            // driveForward(0.45, 900);
            // sleep(100);
            // driveBackward(0.45, 200);

            // intake.frontIntakeMotor.setPower(0);
            // intake.backIntakeMotor.setPower(0);

            // driveBackward(0.55, 800);
            // turnRight(0.50, 400);

            // shoot again (12 total)
            // for (int slot = 1; slot <= 3; slot++) {
            //     transfer.setTransferPos(slot, true);
            //     sleep(300);
            //     transfer.setTransferPos(slot, false);
            //     sleep(150);
            // }

            turret.turretOff();    // turn off flywheel at end
            */

            driveBackward(0.4, 1700);
            scoreArtifacts();
            driveBackward(0.4, 400);
            turnRight(0.5, 360);
            driveForward(0.5, 400);
            intakeArtifacts();
            driveForward(0.5, 1900);
            sleep(300);
            stopIntake();
            extake();
            driveBackward(0.5, 1350);
            stopIntake();
            turnLeft(0.5, 400);
            driveForward(0.4, 400);
            scoreArtifacts();
            strafeRight(0.4, 900);

            transfer.resetAllFlickers();
        }
    }

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

    private void intakeArtifacts() {
        // Run intake motors
        intake.frontIntakeMotor.setPower(1.0);
    }
    private void extake() {
        // Run intake motors
        intake.frontIntakeMotor.setPower(-1.0);
    }

    private void stopIntake() {

        intake.frontIntakeMotor.setPower(0);

    }

    private void scoreArtifacts() {
        // Spin up flywheels
        turret.setTurretPower(-0.75);
        sleep(1000); // Wait for flywheels to spin up
        int slotnum = 0;

        // Flick artifacts one by one
        for (int slot = 1; slot < 4; slot++) {
            if (slot == 1) {slotnum = 2;}
            if (slot == 2) {slotnum = 3;}
            if (slot == 3) {slotnum = 1;}
            transfer.setTransferPos(slotnum, true); // Flick up
            sleep(600); // Wait for artifact to shoot
            transfer.setTransferPos(slotnum, false); // Flick down
            sleep(300); // Small delay between shots
            telemetry.addData("Flicking Slot:", slot);
            telemetry.update();
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