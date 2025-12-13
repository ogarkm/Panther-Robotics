package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Turret PID Tuning", group="Tuning")
public class TurretTuningMain extends LinearOpMode {

    // Hardware
    private CRServo turretServo;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Target
    private int targetTagId = 21;

    // PID parameters
    private double kP = 0.08; // Proportional - main response
    private double kI = 0.0;  // Integral - fixes steady state error (start at 0)
    private double kD = 0.015; // Derivative - reduces overshoot

    // Control limits
    private double maxSpeed = 1.0;
    private double minSpeed = 0.12; // Minimum to overcome friction
    private double deadzone = 1.5; // Stop when within this angle

    // Filtering
    private double lowPassFactor = 0.65;
    private double errorThreshold = 0.2;

    // PID state
    private double filteredError = 0;
    private boolean filterInitialized = false;
    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    // Tuning mode
    private enum TuningParameter {
        KP, KI, KD, MAX_SPEED, MIN_SPEED, DEADZONE, FILTER, ERROR_THRESHOLD
    }
    private TuningParameter selectedParam = TuningParameter.KP;

    // Debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servo
        turretServo = hardwareMap.get(CRServo.class, "turretservo");

        // Initialize AprilTag detection
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  PID TUNING MODE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("\n🎯 Tracking Tag 21");
        telemetry.addLine("\nPID = Proportional + Integral + Derivative");
        telemetry.addLine("P: Main response to error");
        telemetry.addLine("I: Eliminates steady-state error");
        telemetry.addLine("D: Reduces overshoot");
        telemetry.addLine("\nControls:");
        telemetry.addLine("  D-Pad Up/Down: Adjust Value");
        telemetry.addLine("  D-Pad Left/Right: Switch Parameter");
        telemetry.addLine("  Left Bumper: Fine Tune (×0.1)");
        telemetry.addLine("  X: Reset PID (clear integral)");
        telemetry.addLine("  Y: Reset to Defaults");
        telemetry.addLine("  A: Toggle Tag (21/24)");
        telemetry.addLine("\nPress START when ready");
        telemetry.update();

        waitForStart();
        runtime.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            boolean fineTune = gamepad1.left_bumper;
            double multiplier = fineTune ? 0.1 : 1.0;

            // Parameter selection
            if (gamepad1.dpad_left && !lastDpadLeft) {
                selectedParam = getPreviousParam(selectedParam);
            }
            lastDpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right && !lastDpadRight) {
                selectedParam = getNextParam(selectedParam);
            }
            lastDpadRight = gamepad1.dpad_right;

            // Value adjustment
            if (gamepad1.dpad_up && !lastDpadUp) {
                adjustParameter(selectedParam, 0.001 * multiplier);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                adjustParameter(selectedParam, -0.001 * multiplier);
            }
            lastDpadDown = gamepad1.dpad_down;

            // Reset PID
            if (gamepad1.x && !lastX) {
                integral = 0;
                lastError = 0;
                filteredError = 0;
                filterInitialized = false;
            }
            lastX = gamepad1.x;

            // Reset to defaults
            if (gamepad1.y && !lastY) {
                resetToDefaults();
            }
            lastY = gamepad1.y;

            // Toggle target tag
            if (gamepad1.a && !lastA) {
                targetTagId = (targetTagId == 21) ? 24 : 21;
                integral = 0;
                lastError = 0;
                filteredError = 0;
                filterInitialized = false;
            }
            lastA = gamepad1.a;

            // === PID TRACKING LOGIC ===
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0 || dt > 0.1) dt = 0.02; // Cap dt to prevent spikes

            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection targetTag = null;

            for (AprilTagDetection detection : detections) {
                if (detection.id == targetTagId) {
                    targetTag = detection;
                    break;
                }
            }

            if (targetTag != null) {
                double yawDeg = targetTag.ftcPose.yaw;

                // Initialize filter
                if (!filterInitialized) {
                    filteredError = yawDeg;
                    filterInitialized = true;
                }

                // Apply low-pass filter
                filteredError = (lowPassFactor * yawDeg) + ((1 - lowPassFactor) * filteredError);
                double error = filteredError;

                // Ignore tiny noise
                if (Math.abs(error) < errorThreshold) {
                    error = 0;
                }

                if (Math.abs(error) < deadzone) {
                    // Within deadzone - STOP
                    turretServo.setPower(0);
                    integral = 0; // Reset integral to prevent windup
                    lastError = 0;
                    displayTelemetry(targetTag, error, 0, 0, 0, 0, "✓ LOCKED");

                } else {
                    // PID CALCULATION

                    // Proportional term
                    double P = kP * error;

                    // Integral term (accumulate error over time)
                    integral += error * dt;
                    // Anti-windup: limit integral
                    integral = clamp(integral, -5.0, 5.0);
                    double I = kI * integral;

                    // Derivative term (rate of change)
                    double derivative = (error - lastError) / dt;
                    derivative = clamp(derivative, -50, 50); // Limit spikes
                    double D = kD * derivative;

                    lastError = error;

                    // Combine PID terms
                    double pidOutput = P + I + D;

                    // Apply min speed to overcome friction
                    if (Math.abs(pidOutput) > 0.01 && Math.abs(pidOutput) < minSpeed) {
                        pidOutput = Math.copySign(minSpeed, pidOutput);
                    }

                    // Clamp to max speed
                    pidOutput = clamp(pidOutput, -maxSpeed, maxSpeed);

                    // Apply power (negative because servo direction is reversed)
                    turretServo.setPower(-pidOutput);

                    displayTelemetry(targetTag, error, pidOutput, P, I, D, "→ TRACKING");
                }

            } else {
                turretServo.setPower(0);
                filteredError = 0;
                filterInitialized = false;
                integral = 0;
                lastError = 0;
                displayNoTagTelemetry();
            }
        }

        turretServo.setPower(0);
        visionPortal.close();
    }

    private void adjustParameter(TuningParameter param, double delta) {
        switch (param) {
            case KP:
                kP += delta * 10; // Larger increments for P
                kP = clamp(kP, 0.0, 0.5);
                break;
            case KI:
                kI += delta; // Small increments for I
                kI = clamp(kI, 0.0, 0.1);
                break;
            case KD:
                kD += delta * 5; // Medium increments for D
                kD = clamp(kD, 0.0, 0.1);
                break;
            case MAX_SPEED:
                maxSpeed += delta * 10;
                maxSpeed = clamp(maxSpeed, minSpeed, 1.0);
                break;
            case MIN_SPEED:
                minSpeed += delta * 10;
                minSpeed = clamp(minSpeed, 0.0, maxSpeed);
                break;
            case DEADZONE:
                deadzone += delta * 10;
                deadzone = clamp(deadzone, 0.5, 10.0);
                break;
            case FILTER:
                lowPassFactor += delta * 10;
                lowPassFactor = clamp(lowPassFactor, 0.0, 1.0);
                break;
            case ERROR_THRESHOLD:
                errorThreshold += delta * 10;
                errorThreshold = clamp(errorThreshold, 0.0, 3.0);
                break;
        }
    }

    private TuningParameter getNextParam(TuningParameter current) {
        TuningParameter[] params = TuningParameter.values();
        int index = current.ordinal();
        return params[(index + 1) % params.length];
    }

    private TuningParameter getPreviousParam(TuningParameter current) {
        TuningParameter[] params = TuningParameter.values();
        int index = current.ordinal();
        return params[(index - 1 + params.length) % params.length];
    }

    private void resetToDefaults() {
        kP = 0.08;
        kI = 0.0;
        kD = 0.015;
        maxSpeed = 1.0;
        minSpeed = 0.12;
        deadzone = 1.5;
        lowPassFactor = 0.65;
        errorThreshold = 0.2;
        integral = 0;
        lastError = 0;
        filteredError = 0;
        filterInitialized = false;
    }

    private void displayTelemetry(AprilTagDetection tag, double error, double output,
                                  double P, double I, double D, String status) {
        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  🎯 PID TRACKING TAG " + targetTagId);
        telemetry.addLine("═══════════════════════════════");

        telemetry.addLine("\n───── Tracking Status ─────");
        telemetry.addData("✓ Tag", tag.id);
        telemetry.addData("Error", String.format("%.2f°", error));
        telemetry.addData("Distance", String.format("%.1f in", tag.ftcPose.range));
        telemetry.addData("Output Power", String.format("%.3f", output));
        telemetry.addData("Status", status);

        telemetry.addLine("\n───── PID Components ─────");
        telemetry.addData("P term", String.format("%.4f (kP × error)", P));
        telemetry.addData("I term", String.format("%.4f (accumulated)", I));
        telemetry.addData("D term", String.format("%.4f (change rate)", D));
        telemetry.addData("Integral sum", String.format("%.3f", integral));

        telemetry.addLine("\n───── Tuning Parameters ─────");
        displayParameter("kP", kP, selectedParam == TuningParameter.KP);
        displayParameter("kI", kI, selectedParam == TuningParameter.KI);
        displayParameter("kD", kD, selectedParam == TuningParameter.KD);
        displayParameter("Max Speed", maxSpeed, selectedParam == TuningParameter.MAX_SPEED);
        displayParameter("Min Speed", minSpeed, selectedParam == TuningParameter.MIN_SPEED);
        displayParameter("Deadzone", deadzone, selectedParam == TuningParameter.DEADZONE);
        displayParameter("Filter", lowPassFactor, selectedParam == TuningParameter.FILTER);
        displayParameter("Error Thresh", errorThreshold, selectedParam == TuningParameter.ERROR_THRESHOLD);

        telemetry.addLine("\n───── Tuning Guide ─────");
        telemetry.addLine("kP too low: Slow response");
        telemetry.addLine("kP too high: Oscillation");
        telemetry.addLine("kD: Reduces overshoot");
        telemetry.addLine("kI: Fixes steady-state error");

        telemetry.addLine("\n───── Controls ─────");
        telemetry.addLine("D-Pad ↑/↓: Adjust | ←/→: Switch");
        telemetry.addData("Mode", gamepad1.left_bumper ? "FINE (×0.1)" : "NORMAL");

        telemetry.addLine("\n───── Copy to Code ─────");
        telemetry.addLine(String.format("KP = %.4f;", kP));
        telemetry.addLine(String.format("KI = %.5f;", kI));
        telemetry.addLine(String.format("KD = %.4f;", kD));
        telemetry.addLine(String.format("MAX_SPEED = %.2f;", maxSpeed));
        telemetry.addLine(String.format("MIN_SPEED = %.2f;", minSpeed));
        telemetry.addLine(String.format("DEADZONE = %.1f;", deadzone));

        telemetry.update();
    }

    private void displayNoTagTelemetry() {
        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  ✗ NO TAG DETECTED");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("\nLooking for Tag", targetTagId);
        telemetry.addLine("\nPoint camera at AprilTag!");

        telemetry.addLine("\n───── Current Parameters ─────");
        displayParameter("kP", kP, selectedParam == TuningParameter.KP);
        displayParameter("kI", kI, selectedParam == TuningParameter.KI);
        displayParameter("kD", kD, selectedParam == TuningParameter.KD);
        displayParameter("Max Speed", maxSpeed, selectedParam == TuningParameter.MAX_SPEED);
        displayParameter("Min Speed", minSpeed, selectedParam == TuningParameter.MIN_SPEED);
        displayParameter("Deadzone", deadzone, selectedParam == TuningParameter.DEADZONE);
        displayParameter("Filter", lowPassFactor, selectedParam == TuningParameter.FILTER);
        displayParameter("Error Thresh", errorThreshold, selectedParam == TuningParameter.ERROR_THRESHOLD);

        telemetry.update();
    }

    private void displayParameter(String name, double value, boolean selected) {
        String indicator = selected ? ">>> " : "    ";
        String format = name.equals("kI") ? "%.5f" : "%.4f";
        telemetry.addData(indicator + name, String.format(format, value) + (selected ? " <<<" : ""));
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}