// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public class PhoenixUtil {
  private PhoenixUtil() {}
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] = Timer.getFPGATimestamp()
          - 0.02
          + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }
}