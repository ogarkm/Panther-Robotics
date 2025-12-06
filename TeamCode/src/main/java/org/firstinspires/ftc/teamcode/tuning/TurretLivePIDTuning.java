package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Turret Bang-Bang Tuning", group="Tuning")
public class TurretLivePIDTuning extends LinearOpMode {

    // BANG-BANG CONTROL for binary servos (only 1, -1, or 0)
    // The Digital Pulse Holding logic below prevents wind-back on CR Servos.

    private double deadzone = 3.0; // Stop moving when within this many degrees
    private double activationZone = 5.0; // Start moving when beyond this (hysteresis)
    private double lowPassFactor = 0.6; // Smooth out jittery readings

    // Advanced options
    private double errorThreshold = 0.3; // Ignore noise below this
    private double pulseDuration = 50; // ms - how long to pulse when near target
    private boolean usePulseMode = false; // Pulse on/off near target instead of continuous

    // === NEW CONSTANTS FOR DIGITAL BRAKING (HOLDING) ===
    private double BRAKE_PULSE_MS = 10.0; // Tune this: how long to apply full power brake (5ms - 20ms)
    private double BRAKE_WAIT_MS = 100.0; // Tune this: how long to wait between brake pulses (50ms - 200ms)
    private ElapsedTime brakePulseTimer = new ElapsedTime();
    private boolean isBraking = false; // Tracks if we are currently in the pulse ON state

    // Tracking variables
    private double filteredError = 0;
    private boolean filterInitialized = false;
    private boolean isMoving = false; // Track current state for hysteresis
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime pulseTimer = new ElapsedTime();

    // Tuning mode
    private enum TuningParameter {
        DEADZONE, ACTIVATION_ZONE, FILTER, ERROR_THRESHOLD, PULSE_DURATION, PULSE_MODE,
        BRAKE_PULSE, BRAKE_WAIT // <--- NEW PARAMETERS
    }
    private TuningParameter selectedParam = TuningParameter.DEADZONE;

    // Debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap.TurretHwMap turret = new hwMap.TurretHwMap(hardwareMap);

        int targetTagId = 20; // Blue alliance tag

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  TURRET BANG-BANG TUNING");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("\n⚡ CR Servo Control (1/-1/0 only) w/ Digital Hold");
        telemetry.addLine("\n🎯 Tracking Tag 20");
        telemetry.addLine("\nControls:");
        telemetry.addLine("  D-Pad Up: Increase Value");
        telemetry.addLine("  D-Pad Down: Decrease Value");
        telemetry.addLine("  D-Pad Left: Previous Parameter");
        telemetry.addLine("  D-Pad Right: Next Parameter");
        telemetry.addLine("  X: Reset Filter");
        telemetry.addLine("  Y: Reset to Defaults");
        telemetry.addLine("  A: Toggle Pulse Mode");
        telemetry.addLine("  Start: Change to Tag 24 (Red)");
        telemetry.addLine("\nPress START when ready");
        telemetry.update();

        waitForStart();
        loopTimer.reset();
        pulseTimer.reset();

        while (opModeIsActive()) {
            double increment = 0.1; // Fixed increment for all params

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
                adjustParameter(selectedParam, increment);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                adjustParameter(selectedParam, -increment);
            }
            lastDpadDown = gamepad1.dpad_down;

            // Reset filter
            if (gamepad1.x && !lastX) {
                filteredError = 0;
                filterInitialized = false;
                isMoving = false;
                telemetry.addLine("\n✓ Filter Reset!");
            }
            lastX = gamepad1.x;

            // Reset to defaults
            if (gamepad1.y && !lastY) {
                resetToDefaults();
                telemetry.addLine("\n✓ Reset to Defaults!");
            }
            lastY = gamepad1.y;

            // Toggle pulse mode
            if (gamepad1.a) {
                usePulseMode = !usePulseMode;
                sleep(200); // Debounce
            }

            // Toggle target tag
            if (gamepad1.start) {
                targetTagId = (targetTagId == 20) ? 24 : 20;
                filteredError = 0;
                filterInitialized = false;
                isMoving = false;
                sleep(200); // Debounce
            }

            // === BANG-BANG TRACKING LOGIC ===
            AprilTagDetection tag = turret.getAprilTagById(targetTagId);

            double dt = loopTimer.seconds();
            loopTimer.reset();

            if (tag != null) {
                double yawDeg = tag.ftcPose.yaw;

                // Initialize filter on first reading
                if (!filterInitialized) {
                    filteredError = yawDeg;
                    filterInitialized = true;
                }

                // Apply low-pass filter to reduce jitter
                filteredError = (lowPassFactor * yawDeg) + ((1 - lowPassFactor) * filteredError);

                double error = filteredError;

                // Ignore tiny noise
                if (Math.abs(error) < errorThreshold) {
                    error = 0;
                }

                // BANG-BANG CONTROL WITH HYSTERESIS
                double threshold = isMoving ? deadzone : activationZone;

                // >>> START OF FIX: DIGITAL PULSE HOLDING <<<
                if (Math.abs(error) < deadzone) {
                    // Turret is within the acceptable error window (Deadzone).

                    if (isBraking) {
                        // STATE 1: Currently Applying a Brake Pulse

                        if (brakePulseTimer.milliseconds() < BRAKE_PULSE_MS) {
                            // Apply the full power brake pulse (1.0 or -1.0)
                            // If error is POSITIVE (drifted CW), apply NEGATIVE power (CCW)
                            // If error is NEGATIVE (drifted CCW), apply POSITIVE power (CW)
                            double brakePower = (error > 0) ? -1.0 : 1.0;
                            turret.setTurretRotationPower(brakePower);
                            displayTelemetry(tag, error, brakePower, targetTagId, "⚡ BRAKING PULSE");

                        } else {
                            // End of Brake Pulse - Enter Wait State
                            turret.stopTurretRotation(); // Set power to 0.0 (Stop)
                            isBraking = false;
                            brakePulseTimer.reset();
                            displayTelemetry(tag, error, 0.0, targetTagId, "⏸ BRAKE WAIT");
                        }

                    } else {
                        // STATE 2: Waiting or Initiating a Brake Pulse

                        if (Math.abs(error) < deadzone * 0.1) {
                            // If we are very close to the center (10% of deadzone), just wait and stop.
                            turret.stopTurretRotation();
                            displayTelemetry(tag, error, 0.0, targetTagId, "⊙ CENTERED STOP");

                        } else if (brakePulseTimer.milliseconds() > BRAKE_WAIT_MS) {
                            // Wait time is up, and we are not perfectly centered, so start a pulse.
                            isBraking = true;
                            brakePulseTimer.reset();
                            // Next loop iteration will execute the brake pulse (STATE 1)
                            displayTelemetry(tag, error, 0.0, targetTagId, "⏳ STARTING BRAKE");

                        } else {
                            // Still in the wait period after the last pulse
                            turret.stopTurretRotation();
                            displayTelemetry(tag, error, 0.0, targetTagId, "⏸ BRAKE WAIT");
                        }
                    }

                    isMoving = false; // We are in a holding state

                }
                // >>> END OF FIX: DIGITAL PULSE HOLDING <<<

                else if (Math.abs(error) > threshold) {
                    // Outside threshold - MOVE (Original Bang-Bang Tracking)

                    if (usePulseMode && Math.abs(error) < activationZone * 1.5) {
                        // Pulse mode for fine control near target
                        if (pulseTimer.milliseconds() < pulseDuration) {
                            // Pulse ON
                            double power = (error > 0) ? 1.0 : -1.0;
                            turret.setTurretRotationPower(power);
                            isMoving = true;
                            displayTelemetry(tag, error, power, targetTagId, "⚡ PULSING");
                        } else if (pulseTimer.milliseconds() < pulseDuration * 3) {
                            // Pulse OFF (wait 2x duration)
                            turret.stopTurretRotation();
                            displayTelemetry(tag, error, 0, targetTagId, "⏸ PULSE WAIT");
                        } else {
                            pulseTimer.reset();
                        }
                    } else {
                        // Continuous mode - full speed
                        double power = (error > 0) ? 1.0 : -1.0;
                        turret.setTurretRotationPower(power);
                        isMoving = true;
                        displayTelemetry(tag, error, power, targetTagId, "→ TRACKING");
                    }

                } else {
                    // In hysteresis zone - maintain current state (should already be moving if entered from active)
                    if (isMoving) {
                        double power = (error > 0) ? 1.0 : -1.0;
                        turret.setTurretRotationPower(power);
                        displayTelemetry(tag, error, power, targetTagId, "↻ COASTING");
                    } else {
                        turret.stopTurretRotation();
                        displayTelemetry(tag, error, 0, targetTagId, "⊙ HOLD");
                    }
                }

            } else {
                // No tag found - stop
                turret.stopTurretRotation();
                filteredError = 0;
                filterInitialized = false;
                isMoving = false;
                displayNoTagTelemetry(targetTagId);
            }
        }

        turret.stopTurretRotation();
    }

    private void adjustParameter(TuningParameter param, double delta) {
        switch (param) {
            case DEADZONE:
                deadzone += delta;
                deadzone = clamp(deadzone, 0.5, 20.0);
                break;
            case ACTIVATION_ZONE:
                activationZone += delta;
                activationZone = clamp(activationZone, deadzone, 30.0);
                break;
            case FILTER:
                lowPassFactor += delta * 0.1;
                lowPassFactor = clamp(lowPassFactor, 0.0, 1.0);
                break;
            case ERROR_THRESHOLD:
                errorThreshold += delta * 0.1;
                errorThreshold = clamp(errorThreshold, 0.0, 3.0);
                break;
            case PULSE_DURATION:
                pulseDuration += delta * 10;
                pulseDuration = clamp(pulseDuration, 10, 500);
                break;
            case PULSE_MODE:
                usePulseMode = !usePulseMode;
                break;
            // NEW CASES FOR BRAKING
            case BRAKE_PULSE:
                BRAKE_PULSE_MS += delta * 5.0; // Adjust in 5ms steps
                BRAKE_PULSE_MS = clamp(BRAKE_PULSE_MS, 1.0, 50.0);
                break;
            case BRAKE_WAIT:
                BRAKE_WAIT_MS += delta * 10.0; // Adjust in 10ms steps
                BRAKE_WAIT_MS = clamp(BRAKE_WAIT_MS, 20.0, 500.0);
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
        deadzone = 3.0;
        activationZone = 5.0;
        lowPassFactor = 0.6;
        errorThreshold = 0.3;
        pulseDuration = 50;
        usePulseMode = false;
        BRAKE_PULSE_MS = 10.0; // Reset new parameters
        BRAKE_WAIT_MS = 100.0; // Reset new parameters
        filteredError = 0;
        filterInitialized = false;
        isMoving = false;
    }

    private void displayTelemetry(AprilTagDetection tag, double error, double power, int targetId, String status) {
        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  ⚡ BANG-BANG TRACKING");
        telemetry.addLine("═══════════════════════════════");

        telemetry.addLine("\n───── Tracking Status ─────");
        telemetry.addData("✓ Tag Detected", tag.id);
        telemetry.addData("Yaw Error", String.format("%.2f°", error));
        telemetry.addData("Distance", String.format("%.1f in", tag.ftcPose.range));
        telemetry.addData("Servo Power", power == 1 ? "→ FULL CW" : power == -1 ? "← FULL CCW" : "⊙ STOPPED");
        telemetry.addData("Status", status);

        // Visual indicator of error magnitude
        String errorBar = getErrorBar(error);
        telemetry.addData("Error Visual", errorBar);

        telemetry.addLine("\n───── Tuning Parameters ─────");
        displayParameter("Deadzone", deadzone, selectedParam == TuningParameter.DEADZONE, "°");
        displayParameter("Activation", activationZone, selectedParam == TuningParameter.ACTIVATION_ZONE, "°");
        displayParameter("Filter", lowPassFactor, selectedParam == TuningParameter.FILTER, "");
        displayParameter("Noise Thresh", errorThreshold, selectedParam == TuningParameter.ERROR_THRESHOLD, "°");
        displayParameter("Pulse Duration", pulseDuration, selectedParam == TuningParameter.PULSE_DURATION, "ms");
        displayParameter("Pulse Mode", usePulseMode ? 1.0 : 0.0, selectedParam == TuningParameter.PULSE_MODE, usePulseMode ? "ON" : "OFF");
        // NEW TELEMETRY
        displayParameter("Brake Pulse", BRAKE_PULSE_MS, selectedParam == TuningParameter.BRAKE_PULSE, "ms");
        displayParameter("Brake Wait", BRAKE_WAIT_MS, selectedParam == TuningParameter.BRAKE_WAIT, "ms");
        // END NEW TELEMETRY

        telemetry.addLine("\n───── How It Works ─────");
        telemetry.addLine("Deadzone: Stop/Hold when error < this");
        telemetry.addLine("Activation: Start moving when error > this");
        telemetry.addLine("Gap = hysteresis (prevents oscillation)");
        telemetry.addData("Current Zone", Math.abs(error) < deadzone ? "DEAD" :
                Math.abs(error) > activationZone ? "ACTIVE" : "HYSTERESIS");

        telemetry.addLine("\n───── Controls ─────");
        telemetry.addLine("D-Pad ↑/↓: Adjust | ←/→: Switch Param");
        telemetry.addLine("A: Toggle Pulse Mode | Start: Switch Tag");

        telemetry.addLine("\n───── Copy to Code ─────");
        telemetry.addLine(String.format("DEADZONE = %.1f;", deadzone));
        telemetry.addLine(String.format("ACTIVATION_ZONE = %.1f;", activationZone));
        telemetry.addLine(String.format("LOW_PASS_FACTOR = %.2f;", lowPassFactor));
        telemetry.addLine(String.format("ERROR_THRESHOLD = %.2f;", errorThreshold));
        if (usePulseMode) {
            telemetry.addLine(String.format("PULSE_DURATION = %.0f;", pulseDuration));
        }
        // NEW CODE COPY
        telemetry.addLine(String.format("BRAKE_PULSE_MS = %.1f;", BRAKE_PULSE_MS));
        telemetry.addLine(String.format("BRAKE_WAIT_MS = %.1f;", BRAKE_WAIT_MS));
        // END NEW CODE COPY

        telemetry.update();
    }

    private String getErrorBar(double error) {
        int barLength = 20;
        int center = barLength / 2;
        int position = center + (int)(error * 0.5); // Scale error to bar
        position = (int)clamp(position, 0, barLength - 1);

        StringBuilder bar = new StringBuilder();
        for (int i = 0; i < barLength; i++) {
            if (i == center) {
                bar.append("|"); // Center line
            } else if (i == position) {
                bar.append("●"); // Current position
            } else {
                bar.append("-");
            }
        }
        return bar.toString();
    }

    private void displayNoTagTelemetry(int targetId) {
        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("  ✗ NO TAG DETECTED");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("\nLooking for Tag", targetId);
        telemetry.addLine("\nPoint camera at AprilTag!");

        telemetry.addLine("\n───── Current Parameters ─────");
        displayParameter("Deadzone", deadzone, selectedParam == TuningParameter.DEADZONE, "°");
        displayParameter("Activation", activationZone, selectedParam == TuningParameter.ACTIVATION_ZONE, "°");
        displayParameter("Filter", lowPassFactor, selectedParam == TuningParameter.FILTER, "");
        displayParameter("Noise Thresh", errorThreshold, selectedParam == TuningParameter.ERROR_THRESHOLD, "°");
        displayParameter("Pulse Duration", pulseDuration, selectedParam == TuningParameter.PULSE_DURATION, "ms");
        displayParameter("Pulse Mode", usePulseMode ? 1.0 : 0.0, selectedParam == TuningParameter.PULSE_MODE, usePulseMode ? "ON" : "OFF");
        // NEW TELEMETRY
        displayParameter("Brake Pulse", BRAKE_PULSE_MS, selectedParam == TuningParameter.BRAKE_PULSE, "ms");
        displayParameter("Brake Wait", BRAKE_WAIT_MS, selectedParam == TuningParameter.BRAKE_WAIT, "ms");
        // END NEW TELEMETRY

        telemetry.update();
    }

    private void displayParameter(String name, double value, boolean selected, String unit) {
        String indicator = selected ? ">>> " : "    ";
        String valueStr = (unit.equals("ON") || unit.equals("OFF")) ? unit : String.format("%.2f %s", value, unit);
        telemetry.addData(indicator + name, valueStr + (selected ? " <<<" : ""));
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}