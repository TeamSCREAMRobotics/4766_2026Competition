// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.teamscreamrobotics.dashboard.DashboardNumber;

/** Add your docs here. */
public class Dashboard {
  public static DashboardNumber flywheelVelocity;

  private static final String tuning = "Tuning";

  static {
    initialize();
  }

  public static void initialize() {
    flywheelVelocity = new DashboardNumber(tuning, "Flywheel Velocity", 0.0);
  }
}
