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
package com.frc6324.template.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLog;

/** Represents the I/O of a swerve module. */
@FunctionalInterface
public interface ModuleIO {
  /** The loggable inputs for a swerve module. */
  @AutoLog
  class ModuleInputs {
    /** Whether the drive motor is connected. */
    public boolean driveConnected = false;

    /** Whether the steer motor is connected. */
    public boolean steerConnected = false;

    /** Whether the azimuth encoder is connected. */
    public boolean encoderConnected = false;

    /** The drive motor's position. */
    public Angle drivePosition = Radians.zero();

    /** The drive motor's velocity. */
    public AngularVelocity driveVelocity = RadiansPerSecond.zero();

    /** The drive motor's applied voltage. */
    public Voltage driveAppliedVoltage = Volts.zero();

    /** The drive motor's stator current. */
    public Current driveStatorCurrent = Amps.zero();

    /** The drive motor's torque current. */
    public Current driveTorqueCurrent = Amps.zero();

    /** The steer motor's position. */
    public Rotation2d steerPosition = Rotation2d.kZero;

    /** The steer motor's velocity. */
    public AngularVelocity steerVelocity = RadiansPerSecond.zero();

    /** The steer motor's applied voltage. */
    public Voltage steerAppliedVoltage = Volts.zero();

    /** The steer motor's stator current. */
    public Current steerStatorCurrent = Amps.zero();

    /** The steer motor's torque current. */
    public Current steerTorqueCurrent = Amps.zero();

    /** The steer motor's absolute position. */
    public Rotation2d steerAbsolutePosition = Rotation2d.kZero;

    /** The steer motor's absolute velocity. */
    public AngularVelocity steerAbsoluteVelocity = RadiansPerSecond.zero();

    /** The cached odometry drive position values for this module. */
    public double[] odometryDrivePositions = new double[0];

    /** The cached odometry steer position values for this module. */
    public Rotation2d[] odometrySteerPositions = new Rotation2d[0];
  }

  /** Engages the brakes on this module's motors. */
  default void brake() {}

  /** Releases control of this module's motors, allowing them to coast. */
  default void coast() {}

  /**
   * Sets the torque current the drive motor will output.
   *
   * @param output The torque current to output.
   */
  default void setDriveOpenLoop(@NotNull Current output) {}

  /**
   * Sets the voltage the drive motor will output.
   *
   * @param output The voltage to output.
   */
  default void setDriveOpenLoop(@NotNull Voltage output) {}

  /**
   * Sets the torque current the steer motor will output.
   *
   * @param output The torque current to output.
   */
  default void setSteerOpenLoop(@NotNull Current output) {}

  /**
   * Sets the voltage the steer motor will output.
   *
   * @param output The voltage to output.
   */
  default void setSteerOpenLoop(@NotNull Voltage output) {}

  /**
   * Request the drive motor drive at a specific velocity.
   *
   * @param velocity The target velocity.
   * @param acceleration The acceleration used to get to the target velocity.
   * @param torqueFF The torque feedforward to output.
   */
  default void requestDriveVelocity(
      @NotNull AngularVelocity velocity,
      @NotNull AngularAcceleration acceleration,
      @NotNull Torque torqueFF) {}

  /**
   * Request the module to reach a specified state.
   *
   * @param velocity The target drive velocity.
   * @param acceleration The acceleration used to get to the target velocity.
   * @param torqueFF The torque feedforward to output.
   * @param heading The target heading.
   */
  default void requestState(
      @NotNull AngularVelocity velocity,
      @NotNull AngularAcceleration acceleration,
      @NotNull Torque torqueFF,
      @NotNull Rotation2d heading) {
    requestDriveVelocity(velocity, acceleration, torqueFF);
    requestSteerPosition(heading);
  }

  /**
   * Request the steer motor to drive to a specified heading.
   *
   * @param heading The target heading of the swerve module.
   */
  default void requestSteerPosition(@NotNull Rotation2d heading) {}

  /**
   * Updates the loggable inputs for this module.
   *
   * @param inputs The inputs to modify.
   */
  void updateInputs(@NotNull ModuleInputs inputs);
}
