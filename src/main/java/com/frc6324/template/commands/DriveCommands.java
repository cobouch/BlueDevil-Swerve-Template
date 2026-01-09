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

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.frc6324.template.subsystems.drive.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
  static final double ANGLE_TOLERANCE = 2.5;
  static final double POSITION_TOLERANCE = Units.inchesToMeters(2);
  static final double TRANSLATION_MAX_VELOCITY = SwerveDrive.getMaxLinearSpeed();
  static final double TRANSLATION_MAX_ACCELERATION = 7;
  static final double ANGLE_MAX_VELOCITY = SwerveDrive.getMaxAngularSpeed();
  static final double ANGLE_MAX_ACCELERATION = Units.degreesToRadians(720);

  public static final double DEADBAND = 0.1;
  private static final FieldCentric joystickDriveRequest =
      new FieldCentric()
          .withDriveRequestType(SwerveDrive.DRIVE_REQUEST_TYPE)
          .withSteerRequestType(SwerveDrive.STEER_REQUEST_TYPE)
          .withDesaturateWheelSpeeds(true);

  private static final PathConstraints BLINE_CONSTRAINTS =
      new PathConstraints()
          .setEndRotationToleranceDeg(ANGLE_TOLERANCE)
          .setEndTranslationToleranceMeters(POSITION_TOLERANCE)
          .setMaxAccelerationDegPerSec2(ANGLE_MAX_ACCELERATION)
          .setMaxAccelerationMetersPerSec2(TRANSLATION_MAX_ACCELERATION)
          .setMaxVelocityDegPerSec(ANGLE_MAX_VELOCITY)
          .setMaxVelocityMetersPerSec(TRANSLATION_MAX_VELOCITY);

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
                  .times(SwerveDrive.getMaxLinearSpeed());
          Logger.recordOutput("Drive/JoystickDrive/TranslationVector", linearVelocity);

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega *= Math.abs(omega);
          omega *= SwerveDrive.getMaxAngularSpeed();
          Logger.recordOutput("Drive/JoystickDrive/RotationalRate", omega);

          drive.setControl(
              joystickDriveRequest
                  .withVelocityX(linearVelocity.getX())
                  .withVelocityY(linearVelocity.getY())
                  .withRotationalRate(omega));
        });
  }

  public Command driveToPose(SwerveDrive drive, Supplier<Pose2d> poseSupplier) {
    var builder = drive.getBLineBuilder();

    return drive.defer(
        () -> {
          Waypoint target = new Waypoint(poseSupplier.get(), true);

          var path = new Path(target);
          path.setPathConstraints(BLINE_CONSTRAINTS);

          return builder.build(path);
        });
  }

  public Command driveToPose(SwerveDrive drive, Pose2d target) {
    Waypoint targetWaypoint = new Waypoint(target, true);

    var path = new Path(targetWaypoint);
    path.setPathConstraints(BLINE_CONSTRAINTS);

    return drive.getBLineBuilder().build(path);
  }
}
