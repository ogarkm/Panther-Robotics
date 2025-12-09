package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TransferSys;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name="Blue Left Encoder Auto", group="Blue")
public class BlueLeftEncoderAuto extends LinearOpMode {

    private hwMap.DriveHwMap driveTrain;
    private hwMap.IntakeHwMap intake;
    private hwMap.TransferHwMap transfer;
    private hwMap.TurretHwMap turret;

    // power levels - we prolly will set this to max for drive (tune this shit later)
    private static final double DRIVE_POWER = 0.6;
    private static final double STRAFE_POWER = 0.6;
    private static final double TURN_POWER = 0.5;
    private static final double SLOW_POWER = 0.3;

    // encoder constants or smth
    private static final double WHEEL_DIAMETER = Constants.DriveConstants.WHEEL_DIAMETER; // inches
    private static final double TICKS_PER_REVOLUTION = Constants.DriveConstants.TICKS_PER_REVOLUTION;
    private static final double GEAR_RATIO = 1.0; // we need to change this idk the gear ratio yet
    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER); //IS HTIS FORAMULA CORRENCT?

    // tolerance for encoder mvmnts
    private static final int ENCODER_TOLERANCE = 50; // ticks idk prolly needa tune

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new hwMap.DriveHwMap(hardwareMap);
        intake = new hwMap.IntakeHwMap(hardwareMap);
        transfer = new hwMap.TransferHwMap(hardwareMap);
        turret = new hwMap.TurretHwMap(hardwareMap);

        driveTrain.setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Blue Left Encoder Auto Initialized");
        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //sample idk
            driveForward(24, DRIVE_POWER);

            strafeRight(12, STRAFE_POWER); // 12 inches right for testing

            scoreArtifacts();

            driveBackward(18, DRIVE_POWER); // 18 inches backward

            turnLeft(90, TURN_POWER); // 90 degrees left

            intakeArtifacts(2000); // we probably need to change this to turn on and off thing bc we needa have it moving as we move forward

            driveForward(18, DRIVE_POWER);


            strafeLeft(20, DRIVE_POWER);
            driveBackward(10, SLOW_POWER);

            telemetry.addLine("Autonomous boom boom");
            telemetry.update();
        }
    }
//methods look over this

    private void driveForward(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, power, power, power);

            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);

            telemetry.addData("Target", targetTicks);//lowk idk if we need this part but why not
            telemetry.addData("Current", avgPosition);
            telemetry.addData("Remaining", targetTicks - avgPosition);
            telemetry.update();
        }

        stopDrive();
    }

    private void driveBackward(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(-power, -power, -power, -power);

            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);

            telemetry.addData("Target", targetTicks);
            telemetry.addData("Current", avgPosition);
            telemetry.update();
        }

        stopDrive();
    }

    private void strafeRight(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH * 1.4); // strafe correction factor (we need to tune this bc wheels slip in strafingafbabFvfhawbsv)

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, -power, -power, power);

            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);

            telemetry.addData("Strafing Right", "Target: " + targetTicks);
            telemetry.addData("Current", avgPosition);
            telemetry.update();
        }

        stopDrive();
    }

    private void strafeLeft(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH * 1.4);

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(-power, power, power, -power);

            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);

            telemetry.addData("Strafing Left", "Target: " + targetTicks);
            telemetry.addData("Current", avgPosition);
            telemetry.update();
        }

        stopDrive();
    }

    private void turnRight(double degrees, double power) {
        double TURN_TICKS_PER_DEGREE = 10.0; // TUNE THIS VALUE AHIFSHA PLS PLS
        int targetTicks = (int)(degrees * TURN_TICKS_PER_DEGREE);

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, -power, power, -power);

            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);

            telemetry.addData("Turning Right", degrees + "°");
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Current", avgPosition);
            telemetry.update();
        }

        stopDrive();
    }

    private void turnLeft(double degrees, double power) {
        double TURN_TICKS_PER_DEGREE = 10.0; // TUNE THIS VALUE wait honeslty aarush js make this a constant when ur looking over this im too lazy
        int targetTicks = (int)(degrees * TURN_TICKS_PER_DEGREE);

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(-power, power, -power, power);

            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);

            telemetry.addData("Turning Left", degrees + "°");
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Current", avgPosition);
            telemetry.update();
        }

        stopDrive();
    }

    private void stopDrive() {
        driveTrain.stopMotors();
        sleep(100); // Small pause for stability
    }

    //helper methods that idk if we need or not

    private boolean hasReachedTarget(int targetTicks) {
        int[] positions = driveTrain.getEncoderPositions();
        int avgPosition = getAveragePosition(positions);
        return Math.abs(avgPosition) >= Math.abs(targetTicks - ENCODER_TOLERANCE);
    }

    private int getAveragePosition(int[] positions) {
        int sum = 0;
        for (int pos : positions) {
            sum += Math.abs(pos);
        }
        return sum / positions.length;
    }

    //this is the same as our old time uaton

    private void intakeArtifacts(long milliseconds) {
        intake.frontIntakeMotor.setPower(1.0);
        intake.backIntakeMotor.setPower(1.0);
        sleep(milliseconds);
        intake.frontIntakeMotor.setPower(0);
        intake.backIntakeMotor.setPower(0);
        sleep(200);

        for (int i = 1; i <= 3; i++) {
            transfer.detectArtifactColor(i);
        }
    }

    private void scoreArtifacts() {
        // Spin up flywheels
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
//this stuff is claude and wtv it suggested apparently its like advanced for smooth acceleration and stuff idfk
    // it also did like diagonal movement and shit

    private void driveForwardWithPID(double inches, double maxPower) {
        // More advanced PID-controlled encoder movement
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        double kP = 0.001; // Tune these values
        double kI = 0.0;
        double kD = 0.0001;

        double integral = 0;
        double lastError = 0;

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            int[] positions = driveTrain.getEncoderPositions();
            int avgPosition = getAveragePosition(positions);
            int error = targetTicks - avgPosition;

            if (Math.abs(error) < ENCODER_TOLERANCE) {
                break;
            }

            integral += error;
            double derivative = error - lastError;
            lastError = error;

            double power = (kP * error) + (kI * integral) + (kD * derivative);
            power = Math.max(-maxPower, Math.min(maxPower, power));

            // Minimum power to overcome friction
            if (Math.abs(power) < 0.15) {
                power = Math.copySign(0.15, power);
            }

            driveTrain.setMotorPowers(power, power, power, power);

            telemetry.addData("Target", targetTicks);
            telemetry.addData("Current", avgPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power", String.format("%.2f", power));
            telemetry.update();
        }

        stopDrive();
    }

    private void driveDiagonalForwardRight(double inches, double power) {
        // FL and BR move, FR and BL stop (for mecanum diagonal)
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(power, 0, 0, power);

            telemetry.addData("Diagonal Forward-Right", inches + " inches");
            telemetry.update();
        }

        stopDrive();
    }

    private void driveDiagonalForwardLeft(double inches, double power) {
        // FR and BL move, FL and BR stop
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        driveTrain.resetEncoders();
        driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !hasReachedTarget(targetTicks)) {
            driveTrain.setMotorPowers(0, power, power, 0);

            telemetry.addData("Diagonal Forward-Left", inches + " inches");
            telemetry.update();
        }

        stopDrive();
    }
}