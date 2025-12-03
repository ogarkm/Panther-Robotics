package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.teleOp.Constants;

public class DriveTrain {
    private final hwMap.DriveHwMap h_driveTrain;
    public enum DriveState {
        NORMAL,
        TURBO,
        PRECISION,
        STOP
    }
    private DriveState currentState = DriveState.NORMAL;
    private double speedMultiplier = 1.0;
    public DriveTrain(hwMap.DriveHwMap hardware) {
        this.h_driveTrain = hardware;
    }
    public void teleopDrive(double x, double y, double rx) {
        double denominator = ((Math.abs(y) + Math.abs(x) + Math.abs(rx)) > 1 ?
                (Math.abs(y) + Math.abs(x) + Math.abs(rx)) : 1) * speedMultiplier;
        double frontLeftPower = (y + x + rx) * denominator;
        double backLeftPower = (y - x + rx) * denominator;
        double frontRightPower = (y - x - rx) * denominator;
        double backRightPower = (y + x - rx) * denominator;

        h_driveTrain.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public void stop() {
        if (currentState != DriveState.STOP) {
            setDriveState(DriveState.STOP);
        }
        h_driveTrain.stopMotors();
    }
    public double getSpeed() {
        return speedMultiplier;
    }
    public void setDriveState(DriveState state) {
        this.currentState = state;
        switch (state) {
            case NORMAL:
                speedMultiplier = Constants.DriveConstants.NORMAL_SPEED_MULTIPLIER;
                break;
            case PRECISION:
                speedMultiplier = Constants.DriveConstants.PRECISION_SPEED_MULTIPLIER;
                break;
            case TURBO:
                speedMultiplier = Constants.DriveConstants.TURBO_SPEED_MULTIPLIER;
                break;
            case STOP:
                speedMultiplier = Constants.DriveConstants.STOP_SPEED_MULTIPLIER;;
                stop();
                break;
        }
    }
    public DriveState getDriveState() {
        return currentState;
    }
}
