package frc.robot.simulation;

import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.Arena.Simulatable;

import frc.robot.Constants.SimulationConstants;

public class MapleSimFuelPlacer3D {

  private double minX = 0;
  private double minY = 0;
  private double maxY = 0;
  private int numFuel = 0;
  private double currentX = 0;
  private double currentY = 0;

  public MapleSimFuelPlacer3D() {}

  public void placeFuel(double miX, double miY, double maY, int nF) {
    minX = miX;
    minY = miY;
    maxY = maY;
    numFuel = nF;
    currentY = minY;
    currentX = minX;

    for (int i = 0; i < numFuel; i++) {
      //SimulatedArena3D.getInstance().getGamePieceManager().spawnOnField(new RebuiltFuelOnField(q.get(indices[i])));
      if (currentY < maxY) currentY += SimulationConstants.fuelDiameter;
      else {
        currentY = minY;
        currentX += SimulationConstants.fuelDiameter;
      }
    }
  }
}
