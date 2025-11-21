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
package com.frc6324.template.subsystems.drive.odometry;

import com.frc6324.template.subsystems.drive.module.Module;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLog;

public interface OdometryIO {
  /** Inputs that represent the current state of the drivetrain. */
  @AutoLog
  class OdometryInputs {
    public Pose2d robotPose = Pose2d.kZero;
    public Rotation2d robotRotation = Rotation2d.kZero;
    public ChassisSpeeds robotSpeeds = new ChassisSpeeds();
  }

  default void addVisionMeasurement(
      @NotNull Pose2d visionPose, double timestampFPGA, @NotNull Vector<N3> stddevs) {}

  default Optional<Pose2d> samplePoseAtTime(double timestampFPGA) {
    return Optional.empty();
  }

  default double[] getOdometryTimestamps() {
    return new double[0];
  }

  default void setModules(@NotNull Module[] modules) {}

  default void setPose(@NotNull Pose2d pose) {}

  void updateInputs(@NotNull OdometryIO.OdometryInputs inputs);

  default void updateWithTime(
      double timestampFPGA, @NotNull SwerveModulePosition[] modulePositions) {}

  default void updateWithTime(
      double timestampFPGA,
      @NotNull Rotation2d gyroAngle,
      @NotNull SwerveModulePosition[] modulePositions) {}
}
