package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Turret {

    private final hwMap.TurretHwMap hardware;
    private double turretPower = Constants.TurretConstants.TURRET_POWER_MID;
    private int alliance;

    private int[] apriltags = {20, 24};

    private double integral = 0;
    private double lastError = 0;
    private double filteredYaw = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public enum TurretState {
        LAUNCH,
        IDLE,
        EXTAKE,
        TRACKING,
        MANUAL
    }
    private TurretState currentState = TurretState.IDLE;

    public Turret(hwMap.TurretHwMap hardware) {
        this.hardware = hardware;
        loopTimer.reset();
    }

    public void setAlliance(int alliance) {
        this.alliance = alliance;
    }

    public void stop() {
        if (currentState != TurretState.IDLE) {
            setTurretState(TurretState.IDLE);
        }
        hardware.turretOff();
        hardware.stopTurretRotation();
    }

    public void launchTurret() {
        hardware.setTurretPower(turretPower);
    }

    public void setLaunchPower(double power) {
        this.turretPower = power;
    }

    public void setTurretState(TurretState state) {
        this.currentState = state;

        switch (state) {
            case LAUNCH:
                launchTurret();
                break;
            case IDLE:
                hardware.turretOff();
                hardware.stopTurretRotation();
                resetPID();
                break;
            case EXTAKE:
                hardware.setTurretPower(Constants.TurretConstants.EXTAKE_POWER);
                break;
            case TRACKING:
                // Tracking happens in update()
                break;
            case MANUAL:
                // Manual control in update()
                break;
        }
    }

    public TurretState getTurretState() {
        return currentState;
    }

    // Call this every loop in your main TeleOp
    public void update() {
        if (currentState == TurretState.TRACKING) {
            trackAprilTag();
        }
    }

    // Manual rotation control (for testing or teleop override)
    public void manualRotate(double joystickInput) {
        // joystickInput should be -1.0 to 1.0 from right stick X
        double power = joystickInput * Constants.TurretConstants.MAX_ROTATION_POWER;
        hardware.setTurretRotationPower(power);
        resetPID(); // Reset PID when manually controlling
    }

    private void trackAprilTag() {
        AprilTagDetection tag = hardware.getAprilTagById(apriltags[alliance - 1]);

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 0.01;

        if (tag == null) {
            // No target - hold position
            hardware.stopTurretRotation();
            return;
        }

        double yawDeg = tag.ftcPose.yaw;

        // Apply low-pass filter for smoothing
        yawDeg = lowPassFilter(yawDeg, 0.3);

        double error = yawDeg;

        // If within deadzone, stop
        if (Math.abs(error) < Constants.TurretConstants.ROTATION_DEADZONE) {
            hardware.stopTurretRotation();
            resetPID();
            return;
        }

        // PID calculation
        integral += error * dt;
        // Anti-windup: limit integral
        integral = clamp(integral, -10, 10);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidOutput = Constants.TurretConstants.ROTATION_KP * error + Constants.TurretConstants.ROTATION_KI * integral + Constants.TurretConstants.ROTATION_KD * derivative;

        // Add minimum power to overcome static friction
        if (Math.abs(pidOutput) > 0.01) {
            pidOutput = pidOutput + Math.copySign(Constants.TurretConstants.MIN_ROTATION_POWER, pidOutput);
        }

        pidOutput = clamp(pidOutput, -Constants.TurretConstants.MAX_ROTATION_POWER, Constants.TurretConstants.MAX_ROTATION_POWER);

        // Negative because we want to reduce error (turn toward target)
        hardware.setTurretRotationPower(-pidOutput);
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        filteredYaw = 0;
    }

    private double lowPassFilter(double input, double factor) {
        filteredYaw = (factor * input) + (1 - factor) * filteredYaw;
        return filteredYaw;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public void setHoodPos(double pos) {
        hardware.setHoodPos(pos);
    }

    // Getters for telemetry
    public double getLastError() {
        return lastError;
    }
}