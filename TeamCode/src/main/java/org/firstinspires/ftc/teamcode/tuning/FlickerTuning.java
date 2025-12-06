package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hware.hwMap;

@TeleOp(name="Flicker Tuning", group="Diagnostics")
public class FlickerTuning extends LinearOpMode {

    private static final double POSITION_INCREMENT = 0.01; // Fine adjustment
    private static final double POSITION_INCREMENT_FAST = 0.05; // Coarse adjustment

    private int selectedFlicker = 0; // 0 = A, 1 = B, 2 = C
    private double[] positions = {0.5, 0.5, 0.5}; // Start at middle position

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap.TransferHwMap transfer = new hwMap.TransferHwMap(hardwareMap);

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("    FLICKER TUNING MODE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("\nControls:");
        telemetry.addLine("  D-Pad Up: Increase Position");
        telemetry.addLine("  D-Pad Down: Decrease Position");
        telemetry.addLine("  D-Pad Left: Select Previous Flicker");
        telemetry.addLine("  D-Pad Right: Select Next Flicker");
        telemetry.addLine("  Left Bumper: Fine Adjustment Mode");
        telemetry.addLine("  Right Bumper: Coarse Adjustment Mode");
        telemetry.addLine("  X: Set as DOWN position");
        telemetry.addLine("  Y: Set as UP position");
        telemetry.addLine("  A: Test Current Flicker");
        telemetry.addLine("  B: Reset All to 0.5");
        telemetry.addLine("\nPress START when ready");
        telemetry.update();

        waitForStart();

        // Initialize all flickers to starting position
        for (int i = 0; i < 3; i++) {
            transfer.lifts[i].setPosition(positions[i]);
        }

        while (opModeIsActive()) {
            boolean fineMode = gamepad1.left_bumper;
            double increment = fineMode ? POSITION_INCREMENT : POSITION_INCREMENT_FAST;

            // Flicker selection
            if (gamepad1.dpad_left && !lastDpadLeft) {
                selectedFlicker--;
                if (selectedFlicker < 0) selectedFlicker = 2;
            }
            lastDpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right && !lastDpadRight) {
                selectedFlicker++;
                if (selectedFlicker > 2) selectedFlicker = 0;
            }
            lastDpadRight = gamepad1.dpad_right;

            // Position adjustment
            if (gamepad1.dpad_up && !lastDpadUp) {
                positions[selectedFlicker] += increment;
                if (positions[selectedFlicker] > 1.0) positions[selectedFlicker] = 1.0;
                transfer.lifts[selectedFlicker].setPosition(positions[selectedFlicker]);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                positions[selectedFlicker] -= increment;
                if (positions[selectedFlicker] < 0.0) positions[selectedFlicker] = 0.0;
                transfer.lifts[selectedFlicker].setPosition(positions[selectedFlicker]);
            }
            lastDpadDown = gamepad1.dpad_down;

            // Save positions
            if (gamepad1.x && !lastX) {
                telemetry.addLine("\n✓ DOWN Position Saved!");
                telemetry.addData("  Flicker " + getFlickerName(selectedFlicker),
                        String.format("%.3f", positions[selectedFlicker]));
                telemetry.addLine("Add to Constants.TransferConstants:");
                telemetry.addLine(String.format("  FLICK_POS_DOWN = %.3f;", positions[selectedFlicker]));
            }
            lastX = gamepad1.x;

            if (gamepad1.y && !lastY) {
                telemetry.addLine("\n✓ UP Position Saved!");
                telemetry.addData("  Flicker " + getFlickerName(selectedFlicker),
                        String.format("%.3f", positions[selectedFlicker]));
                telemetry.addLine("Add to Constants.TransferConstants:");
                telemetry.addLine(String.format("  FLICK_POS_UP = %.3f;", positions[selectedFlicker]));
            }
            lastY = gamepad1.y;

            // Test flicker
            if (gamepad1.a) {
                testFlicker(transfer, selectedFlicker);
            }

            // Reset all
            if (gamepad1.b) {
                for (int i = 0; i < 3; i++) {
                    positions[i] = 0.5;
                    transfer.lifts[i].setPosition(0.5);
                }
            }

            // Display telemetry
            displayTelemetry(transfer, fineMode, increment);
        }
    }

    private void displayTelemetry(hwMap.TransferHwMap transfer, boolean fineMode, double increment) {
        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("    FLICKER TUNING MODE");
        telemetry.addLine("═══════════════════════════════");

        telemetry.addData("\nAdjustment Mode", fineMode ? "FINE (±0.01)" : "COARSE (±0.05)");
        telemetry.addData("Hold Left Bumper for", "Fine Adjustment");

        telemetry.addLine("\n───── Flicker Positions ─────");

        for (int i = 0; i < 3; i++) {
            String name = getFlickerName(i);
            String indicator = (i == selectedFlicker) ? ">>> " : "    ";
            String color = (i == selectedFlicker) ? "🎯 " : "   ";

            telemetry.addData(color + indicator + "Flicker " + name,
                    String.format("%.3f %s",
                            positions[i],
                            (i == selectedFlicker) ? "<<<" : ""));
        }

        telemetry.addLine("\n───── Quick Commands ─────");
        telemetry.addLine("X: Save as DOWN | Y: Save as UP");
        telemetry.addLine("A: Test Flicker | B: Reset All");

        telemetry.addLine("\n───── Hardware Readback ─────");
        telemetry.addData("Flicker A Actual", String.format("%.3f", transfer.lifts[0].getPosition()));
        telemetry.addData("Flicker B Actual", String.format("%.3f", transfer.lifts[1].getPosition()));
        telemetry.addData("Flicker C Actual", String.format("%.3f", transfer.lifts[2].getPosition()));

        telemetry.addLine("\n───── Current Configuration ─────");
        telemetry.addLine("Copy these to Constants.java:");
        telemetry.addLine(String.format("FLICK_POS_DOWN = %.3f;", positions[selectedFlicker]));
        telemetry.addLine(String.format("FLICK_POS_UP = %.3f;", positions[selectedFlicker]));

        telemetry.update();
    }

    private void testFlicker(hwMap.TransferHwMap transfer, int index) {
        // Move up
        transfer.lifts[index].setPosition(1.0);
        sleep(300);
        // Move down
        transfer.lifts[index].setPosition(0.0);
        sleep(300);
        // Return to tuned position
        transfer.lifts[index].setPosition(positions[index]);
    }

    private String getFlickerName(int index) {
        switch (index) {
            case 0: return "A (Left)";
            case 1: return "B (Top)";
            case 2: return "C (Right)";
            default: return "Unknown";
        }
    }
}