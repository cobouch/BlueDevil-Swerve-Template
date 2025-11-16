/*
 * Copyright (c) 2025 The Blue Devils.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
package org.salemrobotics.frc.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLog;

/** An I/O interface over a gyroscope, which reports the robot's yaw and angular velocity. */
@FunctionalInterface
public interface GyroIO {
  /** Inputs for a gyroscope, containing information such as the robot's yaw. */
  @AutoLog
  class GyroInputs {
    /** Whether the gyroscope is connected. */
    public boolean connected = false;

    /** The reported yaw angle. */
    public Rotation2d yawPosition = Rotation2d.kZero;

    /** The reported angular velocity about the Z axis. */
    public AngularVelocity yawVelocityRadPerSec = RadiansPerSecond.zero();

    /** The cached odometry yaw values for this gyroscope. */
    public Rotation2d[] odometryYawPositions = new Rotation2d[0];
  }

  /**
   * Updates the loggable inputs for this gyroscope.
   *
   * @param inputs The inputs to modify.
   */
  void updateInputs(@NotNull GyroInputs inputs);
}
