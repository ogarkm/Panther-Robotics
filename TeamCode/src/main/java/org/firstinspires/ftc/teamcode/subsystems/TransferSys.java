package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TransferSys {
    private final hwMap.TransferHwMap hardware;
    private final int[] artifactColors = {0, 0, 0};
    private int[] motif = {2, 1, 1};

    private long flickTimer = 0;
    private static final int FLICK_DURATION_MS = 300;
    private List<Integer> currentFlickPlan = new ArrayList<>();
    private int currentFlickIndex = 0;

    public enum TransferState {
        INDEXING,
        FLICKING,
        IDLING,
        STOP,
        MANUAL_FLICK
    }
    private TransferState currentState = TransferState.IDLING;
    private int manualFlickStep = 0;

    public TransferSys(hwMap.TransferHwMap hardware) {
        this.hardware = hardware;
    }

    public void setTransferState(TransferState state) {
        if(this.currentState == state && state == TransferState.FLICKING) return;

        this.currentState = state;

        switch (state) {
            case INDEXING:
                indexAllArtifacts();
                resetAllFlickers();
                break;
            case FLICKING:
                startFlickSequence(); // Start the non-blocking sequence
                break;
            case MANUAL_FLICK:
                resetAllFlickers();
                manualFlickStep = 0;
                break;
            case IDLING:
            case STOP:
                resetAllFlickers();
                break;
        }
    }

    public void update() {
        if (currentState == TransferState.FLICKING) {
            processFlickSequence();
        }
    }

    private void startFlickSequence() {
        indexAllArtifacts();
        currentFlickPlan = buildFlickPlan();

        if (currentFlickPlan.isEmpty()) {
            setTransferState(TransferState.IDLING);
            return;
        }

        currentFlickIndex = 0;
        liftCurrentItem(currentFlickPlan.get(currentFlickIndex));
    }

    private void processFlickSequence() {
        if (System.currentTimeMillis() - flickTimer > FLICK_DURATION_MS) {

            dropCurrentItem(currentFlickPlan.get(currentFlickIndex));

            if (currentFlickIndex < currentFlickPlan.size()) {
                int slot = currentFlickPlan.get(currentFlickIndex);
                artifactColors[slot - 1] = 0;
            }

            currentFlickIndex++;

            if (currentFlickIndex < currentFlickPlan.size()) {
                liftCurrentItem(currentFlickPlan.get(currentFlickIndex)); // Lifts next one immediately as previous drops
            } else {
                // Done
                setTransferState(TransferState.IDLING);
            }
        }
    }

    private void liftCurrentItem(int slot) {
        hardware.setTransferPos(slot, true); // UP
        flickTimer = System.currentTimeMillis();
    }

    private void dropCurrentItem(int slot) {
        hardware.setTransferPos(slot, false); // DOWN
    }

    // --- Helpers ---

    public List<Integer> buildFlickPlan() {
        List<Integer> plan = new ArrayList<>(3);
        boolean[] used = new boolean[3];

        int detectedArtifactCount = 0;
        for (int color : artifactColors) {
            if (color != 0) {
                detectedArtifactCount++;
            }
        }

        for (int desiredColor : motif) {
            for (int slot = 0; slot < 3; slot++) {
                if (!used[slot] && artifactColors[slot] != 0 && artifactColors[slot] == desiredColor) {
                    plan.add(slot + 1);
                    used[slot] = true;
                    break;
                }
            }
        }

        // 2. Remaining Trash
        for (int slot = 0; slot < 3; slot++) {
            if (!used[slot] && artifactColors[slot] != 0) {
                plan.add(slot + 1);
            }
        }
        return plan;
    }

    public void resetAllFlickers() {
        for (int i = 1; i <= 3; i++) {
            hardware.setTransferPos(i, false);
        }
    }

    public void indexAllArtifacts() {
        for (int i = 1; i <= 3; i++) {
            artifactColors[i - 1] = hardware.detectArtifactColor(i);
        }
    }

    public TransferState getTransferState() { return currentState; }
    public void setMotif(int[] newMotif) { this.motif = Arrays.copyOf(newMotif, 3); }

    public void manualFlick() {
        switch (manualFlickStep) {
            case 0: // Flicker A Up
                hardware.setTransferPos(1, true);
                break;
            case 1: // Flicker A Down
                hardware.setTransferPos(1, false);
                break;
            case 2: // Flicker B Up
                hardware.setTransferPos(2, true);
                break;
            case 3: // Flicker B Down
                hardware.setTransferPos(2, false);
                break;
            case 4: // Flicker C Up
                hardware.setTransferPos(3, true);
                break;
            case 5: // Flicker C Down
                hardware.setTransferPos(3, false);
                break;
        }
        manualFlickStep = (manualFlickStep + 1) % 6;
    }
}