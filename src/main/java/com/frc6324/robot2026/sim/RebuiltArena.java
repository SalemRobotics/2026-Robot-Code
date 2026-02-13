package com.frc6324.robot2026.sim;

import com.frc6324.lib.util.DeltaTimeCalculator;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class RebuiltArena extends Arena2026Rebuilt {
  private final List<FlyingFuelSimulation> toRemove = new ArrayList<>();
  private final Set<FlyingFuelSimulation> fuelSimulations = new HashSet<>();
  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  public RebuiltArena() {
    super(false);
  }

  public void addFuelProjectile(FlyingFuelSimulation fuel) {
    fuelSimulations.add(fuel);
  }

  public boolean removeFuelProjectile(FlyingFuelSimulation fuel) {
    return fuelSimulations.remove(fuel);
  }

  @Override
  public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
    final List<Pose3d> poses = super.getGamePiecesPosesByType(type);

    if (type == "Fuel") {
      for (final FlyingFuelSimulation fuel : fuelSimulations) {
        poses.add(fuel.getPose3d());
      }
    }

    return poses;
  }

  @Override
  public synchronized List<GamePiece> getGamePiecesByType(String type) {
    final List<GamePiece> pieces = super.getGamePiecesByType(type);

    if (type == "Fuel") {
      pieces.addAll(fuelSimulations);
    }

    return pieces;
  }

  public Pose3d[] getFuelPoses() {
    return getGamePiecesArrayByType("Fuel");
  }

  @Override
  public synchronized void simulationPeriodic() {
    final double dt = deltaTime.get();

    toRemove.clear();
    for (final FlyingFuelSimulation fuel : fuelSimulations) {
      fuel.update(toRemove, dt);
    }

    for (final FlyingFuelSimulation fuel : toRemove) {
      fuelSimulations.remove(fuel);

      final RebuiltFuelOnField newFuel =
          new RebuiltFuelOnField(fuel.getPose3d().getTranslation().toTranslation2d());
      addGamePiece(newFuel);
    }

    super.simulationPeriodic();
  }

  @Override
  public synchronized boolean removePiece(GamePiece toRemove) {
    if (toRemove instanceof FlyingFuelSimulation fuel) {
      return removeFuelProjectile(fuel);
    }

    return super.removePiece(toRemove);
  }
}
