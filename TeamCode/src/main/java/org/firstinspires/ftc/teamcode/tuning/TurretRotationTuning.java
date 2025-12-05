package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Turret Rotation Tuning", group="Diagnostics")
public class TurretRotationTuning extends LinearOpMode {

    private enum TuningMode {
        MANUAL,
        TRACKING
    }

    private TuningMode currentMode = TuningMode.MANUAL;
    private double manualPower = 0.0;
    private boolean lastModeToggle = false;

    // PID tuning variables
    private double kP = 0.025;
    private double kI = 0.0001;
    private double kD = 0.002;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap.TurretHwMap turret = new hwMap.TurretHwMap(hardwareMap);

        int alliance = 1; // 1 = Blue (tag 20), 2 = Red (tag 24)
        int[] apriltags = {20, 24};

        double integral = 0;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  TURRET ROTATION TUNING");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("\nControls:");
        telemetry.addLine("  Right Stick X: Manual Rotation");
        telemetry.addLine("  A: Toggle Manual/Tracking Mode");
        telemetry.addLine("  Start: Toggle Alliance");
        telemetry.addLine("  D-Pad Up: Increase kP");
        telemetry.addLine("  D-Pad Down: Decrease kP");
        telemetry.addLine("  D-Pad Right: Increase kD");
        telemetry.addLine("  D-Pad Left: Decrease kD");
        telemetry.addLine("  Right Bumper: Fine Tune Mode");
        telemetry.addLine("\nPress START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean fineTune = gamepad1.right_bumper;
            double tuneIncrement = fineTune ? 0.001 : 0.005;

            // Mode toggle
            if (gamepad1.a && !lastModeToggle) {
                currentMode = (currentMode == TuningMode.MANUAL) ?
                        TuningMode.TRACKING : TuningMode.MANUAL;
                integral = 0;
                lastError = 0;
            }
            lastModeToggle = gamepad1.a;

            // Alliance toggle
            if (gamepad1.start) {
                alliance = (alliance == 1) ? 2 : 1;
                sleep(200); // Debounce
            }

            // PID tuning controls
            if (gamepad1.dpad_up) {
                kP += tuneIncrement;
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                kP -= tuneIncrement;
                if (kP < 0) kP = 0;
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                kD += tuneIncrement * 0.1;
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                kD -= tuneIncrement * 0.1;
                if (kD < 0) kD = 0;
                sleep(100);
            }

            // Control logic
            if (currentMode == TuningMode.MANUAL) {
                // Manual control
                manualPower = -gamepad1.right_stick_x * 0.7;
                turret.setTurretRotationPower(manualPower);
                integral = 0;
                lastError = 0;
            } else {
                // Tracking mode
                AprilTagDetection tag = turret.getAprilTagById(apriltags[alliance - 1]);

                if (tag != null) {
                    double yawDeg = tag.ftcPose.yaw;

                    // Calculate dt
                    long currentTime = System.currentTimeMillis();
                    double dt = (currentTime - lastTime) / 1000.0;
                    lastTime = currentTime;
                    if (dt <= 0) dt = 0.01;

                    double error = yawDeg;

                    if (Math.abs(error) < 2.0) {
                        // Within deadzone
                        turret.stopTurretRotation();
                    } else {
                        // PID control
                        integral += error * dt;
                        integral = Math.max(-10, Math.min(10, integral)); // Anti-windup

                        double derivative = (error - lastError) / dt;
                        lastError = error;

                        double pidOutput = kP * error + kI * integral + kD * derivative;

                        // Add minimum power to overcome friction
                        if (Math.abs(pidOutput) > 0.01) {
                            pidOutput = pidOutput + Math.copySign(0.12, pidOutput);
                        }

                        pidOutput = Math.max(-0.7, Math.min(0.7, pidOutput));

                        turret.setTurretRotationPower(-pidOutput);
                    }
                } else {
                    // No tag found
                    turret.stopTurretRotation();
                    integral = 0;
                    lastError = 0;
                }
            }

            // Telemetry
            displayTelemetry(turret, alliance, apriltags, fineTune);
        }

        turret.stopTurretRotation();
    }

    private void displayTelemetry(hwMap.TurretHwMap turret, int alliance, int[] apriltags, boolean fineTune) {
        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  TURRET ROTATION TUNING");
        telemetry.addLine("═══════════════════════════════");

        telemetry.addData("\nMode", currentMode == TuningMode.MANUAL ? "MANUAL" : "TRACKING");
        telemetry.addData("Alliance", alliance == 1 ? "BLUE (Tag 20)" : "RED (Tag 24)");
        telemetry.addData("Tune Mode", fineTune ? "FINE (±0.001)" : "COARSE (±0.005)");

        if (currentMode == TuningMode.MANUAL) {
            telemetry.addData("\nManual Power", String.format("%.2f", manualPower));
            telemetry.addLine("\nUse right stick X to rotate");
        } else {
            telemetry.addLine("\n───── PID Values ─────");
            telemetry.addData("kP", String.format("%.4f", kP));
            telemetry.addData("kI", String.format("%.5f", kI));
            telemetry.addData("kD", String.format("%.4f", kD));

            telemetry.addLine("\n───── AprilTag Tracking ─────");
            AprilTagDetection tag = turret.getAprilTagById(apriltags[alliance - 1]);

            if (tag != null) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Yaw Error", String.format("%.2f°", tag.ftcPose.yaw));
                telemetry.addData("Distance", String.format("%.1f inches", tag.ftcPose.range));
                telemetry.addData("Status", "✓ TRACKING");
            } else {
                telemetry.addData("Status", "✗ NO TAG DETECTED");
                telemetry.addLine("Point camera at AprilTag " + apriltags[alliance - 1]);
            }
        }

        telemetry.addLine("\n───── Instructions ─────");
        telemetry.addLine("D-Pad Up/Down: Adjust kP");
        telemetry.addLine("D-Pad Right/Left: Adjust kD");
        telemetry.addLine("A: Toggle Mode | Start: Alliance");
        telemetry.addLine("\nCopy these to Constants.java:");
        telemetry.addLine(String.format("ROTATION_KP = %.4f;", kP));
        telemetry.addLine(String.format("ROTATION_KD = %.4f;", kD));

        telemetry.update();
    }
}