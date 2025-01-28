// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.simulation;

import java.util.ArrayDeque;
import java.util.List;
import java.util.Queue;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rectangle;
import org.dyn4j.world.listener.ContactListener;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class ElevatorIntakeSimulation {
    private final int capacity;
    private int gamePiecesInIntakeCount;
    private boolean intakeRunning;

    private final Queue<GamePieceOnFieldSimulation> gamePiecesToRemove;
    private final AbstractDriveTrainSimulation driveTrainSimulation;
    private final String targetedGamePieceType;
    private final Supplier<Double> heightSupplier;
    private final Rectangle collisionBox;

    public ElevatorIntakeSimulation(
        String targetedGamePieceType,
        AbstractDriveTrainSimulation driveTrainSimulation,
        Supplier<Double> heightSupplier,
        Rectangle collisionBox,
        int capacity) {
        this.targetedGamePieceType = targetedGamePieceType;
        this.driveTrainSimulation = driveTrainSimulation;
        this.heightSupplier = heightSupplier;
        this.collisionBox = collisionBox;
        this.capacity = capacity;

        this.gamePiecesToRemove = new ArrayDeque<>(capacity);

        this.gamePiecesInIntakeCount = 0;
        this.intakeRunning = false;
    }

    public void update() {
        List<Pose3d> gamePieces = SimulatedArena.getInstance().getGamePiecesByType(targetedGamePieceType);
        for (Pose3d gamePiece : gamePieces) {
            //if (collisionBox.contains(gamePiece.getTranslation().getX(), gamePiece.getTranslation().getY())) {
            //    gamePiecesToRemove.add(new GamePieceOnFieldSimulation(gamePiece));
            //}
        }
    }

    public boolean isGamePieceInIntake(String gamePieceType) {

        return gamePiecesInIntakeCount > 0;
    }

    public void startIntake() {
        if (intakeRunning) return;

        this.intakeRunning = true;
    }

    public void stopIntake() {
        if (!intakeRunning) return;

        this.intakeRunning = false;
    }

    public int getGamePiecesAmount() {
        return gamePiecesInIntakeCount;
    }

    public boolean obtainGamePieceFromIntake() {
        if (gamePiecesInIntakeCount < 1) return false;
        gamePiecesInIntakeCount--;
        return true;
    }
}
