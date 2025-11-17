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
package org.salemrobotics.frc.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.val;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;
import org.salemrobotics.frc.subsystems.drive.SwerveDrive;

public final class DriveCommands {
  public static final double DEADBAND = 0.1;
  private static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(8, 20);
  private static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4.73, 5);

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
          @NotNull SwerveDrive drive,
          DoubleSupplier xSupplier,
          DoubleSupplier ySupplier,
          DoubleSupplier omegaSupplier) {
    return drive.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                      .times(drive.getMaxLinearSpeed().in(MetersPerSecond));

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega *= Math.abs(omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                      linearVelocity.getX(),
                      linearVelocity.getY(),
                  omega * drive.getMaxAngularSpeed().in(RadiansPerSecond));

          drive.runAllianceCentric(speeds);
        });
  }

  private static Pose2d autoStartingPose = Pose2d.kZero;
  private static Command driveToAuto = null;

  public static Command driveToAutoStart(@NotNull SwerveDrive drive, Supplier<Command> autoSupplier) {
      return drive.defer(() -> {
        val cmd = autoSupplier.get();
        if (cmd instanceof PathPlannerAuto auto) {
            autoStartingPose = auto.getStartingPose();

            Logger.recordOutput("AutoAlign/DriveToPose/AutoName", auto.getName());

            if (driveToAuto == null) {
                driveToAuto = new DriveToPoseCommand(drive, "DriveToAuto", () -> autoStartingPose);
            }

            return driveToAuto;
        }

        // No auto to drive to
        return Commands.none();
      });
  }
}
