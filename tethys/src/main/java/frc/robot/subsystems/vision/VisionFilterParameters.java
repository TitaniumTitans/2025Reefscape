/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

// Parameters to filter vision measurements
public record VisionFilterParameters(
    double xyStandardDevBase,
    double rotStandardDevBase,
    Distance aprilTagWidth,
    double maxAmbiguityRatio,
    Rotation2d estimatedFOV,
    Distance fieldWidth,
    Distance fieldLength,
    Distance zMargin) {

  public VisionFilterParameters {}
}