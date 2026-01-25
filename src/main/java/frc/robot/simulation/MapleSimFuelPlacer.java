package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SimulationConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class MapleSimFuelPlacer {

  private double minX = 0;
  private double minY = 0;
  private double maxY = 0;
  private int numFuel = 0;
  private double currentX = 0;
  private double currentY = 0;

  public MapleSimFuelPlacer() {}

  public void placeFuel(double miX, double miY, double maY, int nF) {
    minX = miX;
    minY = miY;
    maxY = maY;
    numFuel = nF;
    currentY = minY;
    currentX = minX;

    for (int i = 0; i < numFuel; i++) {
      SimulatedArena.getInstance()
          .addGamePiece(new RebuiltFuelOnField(new Translation2d(currentX, currentY)));
      if (currentY < maxY) currentY += SimulationConstants.fuelDiameter;
      else {
        currentY = minY;
        currentX += SimulationConstants.fuelDiameter;
      }
    }
  }
}
