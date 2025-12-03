// #---------------------------#
//  Basic Mecanum TeleOP
// 
//  Adapted from Game Manual 0
//  https://gm0.org
//  
// #---------------------------#

package org.firstinspires.ftc.teamcode.teleOp_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanum_Strafe_Test", group="TEST")
public class BasicTeleOP extends LinearOpMode {

    hwMap hw = new hwMap(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            drive(x, y, rx);
        }
    }

    private void drive(double x, double y, double rx) {
        // Mecanum wheel calculations
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * 2;
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set motor powers
        hw.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}