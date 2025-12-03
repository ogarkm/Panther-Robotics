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

    // Updated PID and System Constants from reference
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double MAX_TURRET_DEG = 250;
    private static final double SERVO_RANGE_DEG = 300;

    private static final double KP = 0.015;
    private static final double KI = 0.0001;
    private static final double KD = 0.0015;

    private static final double MAX_PID_OUTPUT = 5;
    private static final double DEADZONE = 1.2;

    private double turretAngleDeg = 0;
    private double integral = 0;
    private double lastError = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public enum TurretState {
        LAUNCH,
        IDLE,
        EXTAKE,
        RESET
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
    }

    public void launchTurret() {
        hardware.setTurretPower(turretPower);
    }

    public void setLaunchPower(double power) {
        this.turretPower = power;
    }

    private void resetTurret() {
        // TODO - Reset Whatever
    }

    public void setTurretState(TurretState state) {
        this.currentState = state;

        switch (state) {
            case LAUNCH:
                launchTurret();
                break;
            case IDLE:
                hardware.turretOff();
                break;
            case EXTAKE:
                hardware.setTurretPower(Constants.TurretConstants.EXTAKE_POWER);
                break;
            case RESET:
                resetTurret();
                hardware.turretOff();
                break;
        }
    }

    public TurretState getTurretState() {
        return currentState;
    }

    // --- New Tracking Logic ---

    public void update() {
        AprilTagDetection tag = hardware.getAprilTagById(apriltags[alliance - 1]);

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 0.01;

        if (tag == null) {
            holdPosition();
            return;
        }

        double yawDeg = tag.ftcPose.yaw;

        yawDeg = lowPass(yawDeg, 0.3);

        double error = yawDeg;

        if (Math.abs(error) < DEADZONE) error = 0;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = KP * error + KI * integral + KD * derivative;
        pid = clamp(pid, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

        turretAngleDeg += pid;
        if (Math.abs(turretAngleDeg) > MAX_TURRET_DEG) {
            if (turretAngleDeg > 0) turretAngleDeg -= 360;
            else turretAngleDeg += 360;
        }

        setServoToAngle(turretAngleDeg);
    }

    private void setServoToAngle(double angleDeg) {
        double normalized = angleDeg / SERVO_RANGE_DEG;
        while (normalized < 0) normalized += 1;
        while (normalized > 1) normalized -= 1;
        normalized = clamp(normalized, SERVO_MIN, SERVO_MAX);
        hardware.setTurretPos(normalized);
    }

    private void holdPosition() {
        setServoToAngle(turretAngleDeg);
    }

    private double lowPass(double input, double factor) {
        return (factor * input) + (1 - factor) * lastError;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}