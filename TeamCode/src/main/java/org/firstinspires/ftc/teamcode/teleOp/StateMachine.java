package org.firstinspires.ftc.teamcode.teleOp;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TransferSys;

enum RobotState {
    INIT,
    TELEOP,
    ESTOP
}
enum GameState {
    // Generic states that work for any season
    INTAKING,           // Collecting game elements
    EXTAKING,
    SCORING,            // Scoring in goals/backboard
    LIFTING,           // Hanging/Climbing
    IDLE
}


public class StateMachine {

    private RobotState currentRobotState = RobotState.INIT;
    private GameState currentGameState = GameState.IDLE;
    private final DriveTrain m_driveTrain;
    private final Intake m_intake;
    private final Lift m_lift;

    private final TransferSys m_transfer;


    public StateMachine(hwMap.LiftHwMap h_lift, hwMap.DriveHwMap h_driveTrain, hwMap.IntakeHwMap h_intake, hwMap.TransferHwMap h_transfer) {
        this.m_driveTrain = new DriveTrain(h_driveTrain);
        this.m_intake = new Intake(h_intake);
        this.m_lift = new Lift(h_lift, h_driveTrain);
        this.m_transfer = new TransferSys(h_transfer);

        setRobotState(RobotState.INIT);
        setGameState(GameState.IDLE);
    }

    public void setRobotState(RobotState newState) {
        if (this.currentRobotState == newState) return;

        handleRobotStateExit(this.currentRobotState);
        this.currentRobotState = newState;
        handleRobotStateEntry(newState);
    }

    public void setGameState(GameState newState) {
        if (this.currentGameState == newState) return;

        handleGameStateExit(this.currentGameState);
        this.currentGameState = newState;
        handleGameStateEntry(newState);
    }

    private void handleGameStateExit(GameState oldState) {
        m_intake.setIntakeState(Intake.IntakeState.IDLE);
        m_lift.setLiftState(Lift.LiftState.IDLE);
        switch (oldState) {
            case INTAKING:
                m_transfer.setTransferState(TransferSys.TransferState.INDEXING);
                break;
            case SCORING:
                m_transfer.setTransferState(TransferSys.TransferState.STOP);
                break;
        }
    }

    private void handleGameStateEntry(GameState newState) {
        switch (newState) {
            case INTAKING:
                m_driveTrain.setDriveState(DriveTrain.DriveState.NORMAL);
                m_intake.setIntakeState(Intake.IntakeState.INTAKE);
                break;
            case EXTAKING:
                m_driveTrain.setDriveState(DriveTrain.DriveState.NORMAL);
                m_intake.setIntakeState(Intake.IntakeState.EXTAKE);
                break;
            case SCORING:
                m_driveTrain.setDriveState(DriveTrain.DriveState.PRECISION);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                m_transfer.setTransferState(TransferSys.TransferState.FLICKING);
                break;
            case LIFTING:
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
            case IDLE:
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                break;
        }
    }
    private void handleRobotStateExit(RobotState oldState) {
        switch (oldState) {
            case TELEOP:
                m_driveTrain.stop();
                break;
            case ESTOP:
                break;
        }
    }

    private void handleRobotStateEntry(RobotState newState) {
        switch (newState) {
            case TELEOP:
                m_driveTrain.setDriveState(DriveTrain.DriveState.NORMAL);
                m_intake.setIntakeState(Intake.IntakeState.IDLE);
                m_lift.setLiftState(Lift.LiftState.LIFT_DOWN);
                break;
            case ESTOP:
                m_driveTrain.setDriveState(DriveTrain.DriveState.STOP);
                m_intake.setIntakeState(Intake.IntakeState.STOP);
                break;
        }
    }

    private void handleStatePeriodic() {
        switch (currentGameState) {
            case INTAKING:
                // Run intake logic, check if we have element, etc.
                break;
            case SCORING:
                // Run scoring sequence, check alignment, etc.
                break;
            case LIFTING:
                // Climbing sequence, check hooks, etc.
                break;
        }
    }
    public void emergencyStop() {
        setRobotState(RobotState.ESTOP);
        setGameState(GameState.IDLE);
    }
    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
    public Intake getIntake() {                            
        return m_intake;
    }
    public RobotState getCurrentRobotState() {
        return currentRobotState;
    }
}