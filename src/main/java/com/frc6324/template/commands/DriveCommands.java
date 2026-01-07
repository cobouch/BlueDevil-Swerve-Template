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
package com.frc6324.template.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.frc6324.template.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
  public static final double DEADBAND = 0.1;
  private static final FieldCentric joystickDriveRequest =
      new FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withDesaturateWheelSpeeds(true);

  @Contract(pure = true)
  private DriveCommands() {}

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for finer control
    magnitude *= magnitude;

    return new Translation2d(magnitude, 0).rotateBy(direction);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  @Contract("_, _, _, _ -> new")
  public static @NotNull Command joystickDrive(
      SwerveDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(drive.getMaxLinearSpeed().in(MetersPerSecond));
          Logger.recordOutput("Drive/JoystickDrive/TranslationVector", linearVelocity);

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega *= Math.abs(omega);
          Logger.recordOutput("Drive/JoystickDrive/RotationalRate", omega);

          drive.setControl(
              joystickDriveRequest
                  .withVelocityX(linearVelocity.getX())
                  .withVelocityY(linearVelocity.getY())
                  .withRotationalRate(omega));
        });
  }
}
