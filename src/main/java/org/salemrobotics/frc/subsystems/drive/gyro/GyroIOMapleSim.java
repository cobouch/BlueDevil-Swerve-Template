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

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.jetbrains.annotations.NotNull;

/**
 * An I/O implementation of a gyroscope on the robot using maple-sim's {@link GyroSimulation}
 * produced by {@link SwerveDriveSimulation#getGyroSimulation()}
 *
 * @param simulation The gyro simulation for the drivetrain.
 */
public record GyroIOMapleSim(GyroSimulation simulation) implements GyroIO {
  @Override
  public void updateInputs(@NotNull GyroInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = simulation.getGyroReading();
    inputs.yawVelocityRadPerSec = simulation.getMeasuredAngularVelocity();
    inputs.odometryYawPositions = simulation.getCachedGyroReadings();
  }
}
