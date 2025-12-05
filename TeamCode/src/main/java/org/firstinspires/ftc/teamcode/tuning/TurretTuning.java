package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.subsystems.Turret;



@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuning extends LinearOpMode {

    private hwMap.TurretHwMap h_turret;
    private Turret turret;

    private int alliance = 1; // 1 for Blue, 2 for Red
    private boolean wasStartPressed = false;
    private boolean wasAPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        h_turret = new hwMap.TurretHwMap(hardwareMap);
        turret = new Turret(h_turret);

        turret.setAlliance(alliance);
        turret.setTurretState(Turret.TurretState.MANUAL); // Start in manual mode

        waitForStart();

        while (opModeIsActive()) {
            // Alliance switching
            boolean isStartPressed = gamepad1.start;
            if (isStartPressed && !wasStartPressed) {
                alliance = (alliance == 1) ? 2 : 1;
                turret.setAlliance(alliance);
            }
            wasStartPressed = isStartPressed;

            // Mode switching (Manual vs Tracking)
            boolean isAPressed = gamepad1.a;
            if (isAPressed && !wasAPressed) {
                if (turret.getTurretState() == Turret.TurretState.MANUAL) {
                    turret.setTurretState(Turret.TurretState.TRACKING);
                } else {
                    turret.setTurretState(Turret.TurretState.MANUAL);
                }
            }
            wasAPressed = isAPressed;


            // Control based on state
            if (turret.getTurretState() == Turret.TurretState.MANUAL) {
                double manualPower = gamepad1.right_stick_x;
                turret.manualRotate(manualPower);
            } else if (turret.getTurretState() == Turret.TurretState.TRACKING) {
                turret.update();
            }



            // Telemetry
            telemetry.addData("Turret Mode", turret.getTurretState());
            telemetry.addData("Alliance", alliance == 1 ? "Blue" : "Red");
            telemetry.addData("Press 'A' to switch mode", "");
            telemetry.addData("Press 'Start' to switch alliance", "");
            if (turret.getTurretState() == Turret.TurretState.MANUAL) {
                telemetry.addData("Manual Control", "Right Stick X");
            } else {
                telemetry.addData("Tracking Error", turret.getLastError());
            }

            telemetry.update();
        }

        turret.stop();
    }
}
