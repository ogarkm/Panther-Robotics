package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.teleOp.Constants;

public abstract class AutonConfig extends LinearOpMode {

    protected hwMap.DriveHwMap driveTrain;
    protected hwMap.IntakeHwMap intake;
    protected hwMap.TransferHwMap transfer;
    protected hwMap.TurretHwMap turret;

    protected static final double DRIVE_POWER = 0.6;
    protected static final double STRAFE_POWER = 0.6;
    protected static final double TURN_POWER = 0.5;
    protected static final double SLOW_POWER = 0.3;

    protected static final double SplinePower = 0.6;

    protected static final double WHEEL_DIAMETER = Constants.DriveConstants.WHEEL_DIAMETER;
    protected static final double TICKS_PER_REVOLUTION = Constants.DriveConstants.TICKS_PER_REVOLUTION;
    protected static final double GEAR_RATIO = 1.0;
    protected static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER);
    protected static final int ENCODER_TOLERANCE = 50;

    // Constant to tune how "aggressive" the turn is relative to distance in splines
    private static final double TURN_SENSITIVITY = 10.0;

    public void initRobot() {
        driveTrain = new hwMap.DriveHwMap(hardwareMap);
        intake = new hwMap.IntakeHwMap(hardwareMap);
        transfer = new hwMap.TransferHwMap(hardwareMap);
        turret = new hwMap.TurretHwMap(hardwareMap);

        driveTrain.setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Standard Movements ---

    protected void driveForward(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, power, power, power);
            updateTelemetry("Forward", targetTicks);
        }
        stopDrive();
    }

    protected void driveBackward(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(-power, -power, -power, -power);
            updateTelemetry("Backward", targetTicks);
        }
        stopDrive();
    }

    protected void strafeRight(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH * 1.4);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, -power, -power, power);
            updateTelemetry("Strafe Right", targetTicks);
        }
        stopDrive();
    }

    protected void strafeLeft(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH * 1.4);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(-power, power, power, -power);
            updateTelemetry("Strafe Left", targetTicks);
        }
        stopDrive();
    }

    protected void turnRight(double degrees, double power) {
        int targetTicks = (int)(degrees * TURN_SENSITIVITY);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, -power, power, -power);
            updateTelemetry("Turn Right", targetTicks);
        }
        stopDrive();
    }

    protected void turnLeft(double degrees, double power) {
        int targetTicks = (int)(degrees * TURN_SENSITIVITY);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(-power, power, -power, power);
            updateTelemetry("Turn Left", targetTicks);
        }
        stopDrive();
    }

    // --- Splining Methods ---

    protected void driveSpline(double forwardInches, double strafeInches, double turnDegrees, double maxPower) {
        // 1. Calculate the hypothetical distance the robot actually travels across the floor
        double translationDistance = Math.hypot(forwardInches, strafeInches);

        // 2. Convert to ticks
        int targetTicks = (int)(translationDistance * TICKS_PER_INCH);

        // 3. Reset Encoders
        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // 4. Calculate Vector Powers
        // We normalize these so the robot performs all actions proportionally
        double x = strafeInches * 1.4; // Strafe friction correction
        double y = forwardInches;

        double rx = -turnDegrees * 0.02;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > maxPower) {
            fl = (fl / max) * maxPower;
            fr = (fr / max) * maxPower;
            bl = (bl / max) * maxPower;
            br = (br / max) * maxPower;
        }

        // 5. Run the loop
        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(fl, fr, bl, br);

            telemetry.addData("Spline Status", "Running");
            telemetry.addData("Dist Target", targetTicks);
            telemetry.addData("Current", getAveragePosition(driveTrain.getEncoderPositions()));
            telemetry.update();
        }
        stopDrive();
    }
    // --- Spline Right: Drive Forward + Strafe Right + Turn Right ---
    protected void splineRight(double forwardInches, double strafeInches, double turnDegrees, double power) {
        driveSpline(forwardInches, Math.abs(strafeInches), Math.abs(turnDegrees), power);
    }

    //  --- Spline Left: Drive Forward + Strafe Left + Turn Left ---
    protected void splineLeft(double forwardInches, double strafeInches, double turnDegrees, double power) {
        driveSpline(forwardInches, -Math.abs(strafeInches), -Math.abs(turnDegrees), power);
    }

    // --- Subsystem Methods ---

    protected void intakeArtifacts() {
        intake.frontIntakeMotor.setPower(1.0);
        intake.backIntakeMotor.setPower(1.0);
    }

    protected void stopIntake() {
        intake.frontIntakeMotor.setPower(0);
        intake.backIntakeMotor.setPower(0);
        sleep(200);

        for (int i = 1; i <= 3; i++) {
            transfer.detectArtifactColor(i);
        }
    }

    protected void scoreArtifacts() {
        turret.setTurretPower(0.7);
        sleep(1000);

        for (int slot = 1; slot <= 3; slot++) {
            transfer.setTransferPos(slot, true);
            sleep(300);
            transfer.setTransferPos(slot, false);
            sleep(200);
        }

        turret.turretOff();
        sleep(300);
    }

    // --- PID / Diagonal ---

    protected void driveForwardWithPID(double inches, double maxPower) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        double kP = 0.001;
        double kI = 0.0;
        double kD = 0.0001;
        double integral = 0;
        double lastError = 0;

        resetAndRunEncoders();

        while (opModeIsActive()) {
            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);
            int error = targetTicks - avgPosition;

            if (Math.abs(error) < ENCODER_TOLERANCE) break;

            integral += error;
            double derivative = error - lastError;
            lastError = error;

            double power = (kP * error) + (kI * integral) + (kD * derivative);
            power = Range.clip(power, -maxPower, maxPower);

            // Deadzone to prevent oscillation
            if (Math.abs(power) < 0.15 && Math.abs(error) > ENCODER_TOLERANCE) {
                power = Math.copySign(0.15, power);
            }

            driveTrain.setMotorPowers(power, power, power, power);
            telemetry.addData("PID Error", error);
            telemetry.update();
        }
        stopDrive();
    }

    protected void driveDiagonalForwardRight(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, 0, 0, power);
            telemetry.addData("Diagonal FR", targetTicks);
            telemetry.update();
        }
        stopDrive();
    }

    protected void driveDiagonalForwardLeft(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        resetAndRunEncoders();

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(0, power, power, 0);
            telemetry.addData("Diagonal FL", targetTicks);
            telemetry.update();
        }
        stopDrive();
    }

    // --- Helpers ---

    private void resetAndRunEncoders() {
        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void stopDrive() {
        driveTrain.stopMotors();
        sleep(100);
    }

    protected boolean hasReachedTarget(int targetTicks) {
        int[] positions = driveTrain.getEncoderPositions();
        int avgPosition = getAveragePosition(positions);
        return Math.abs(avgPosition) >= Math.abs(targetTicks - ENCODER_TOLERANCE);
    }

    protected int getAveragePosition(int[] positions) {
        int sum = 0;
        for (int pos : positions) {
            sum += Math.abs(pos);
        }
        return sum / positions.length;
    }

    private void updateTelemetry(String action, int target) {
        int[] positions = driveTrain.getEncoderPositions();
        int avgPosition = getAveragePosition(positions);
        telemetry.addData("Action", action);
        telemetry.addData("Target", target);
        telemetry.addData("Current", avgPosition);
        telemetry.update();
    }
}