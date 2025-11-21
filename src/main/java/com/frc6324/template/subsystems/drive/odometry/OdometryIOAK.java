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

import static com.frc6324.template.subsystems.drive.DrivetrainConstants.*;

import com.frc6324.template.subsystems.drive.module.Module;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import lombok.val;
import org.jetbrains.annotations.NotNull;

public class OdometryIOAK implements OdometryIO {
  private final Rotation2d rawGyroPosition = Rotation2d.kZero;
  private final SwerveModulePosition[] lastModulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          KINEMATICS,
          rawGyroPosition,
          lastModulePositions,
          Pose2d.kZero,
          AK_ODOMETRY_STDDEVS,
          VecBuilder.fill(0, 0, 0));

  private Module[] modules = new Module[0];

  @Override
  public void addVisionMeasurement(
      @NotNull Pose2d visionPose, double timestampFPGA, @NotNull Vector<N3> stddevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampFPGA, stddevs);
  }

  @Override
  public double[] getOdometryTimestamps() {
    return AKOdometryThread.getTimestamps();
  }

  @Override
  public Optional<Pose2d> samplePoseAtTime(double timestampFPGA) {
    try {
      return poseEstimator.sampleAt(timestampFPGA);
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  @Override
  public void setModules(@NotNull Module[] modules) {
    this.modules = modules;
  }

  @Override
  public void setPose(@NotNull Pose2d pose) {
    poseEstimator.resetPosition(rawGyroPosition, lastModulePositions, pose);
  }

  @Override
  public void updateInputs(@NotNull OdometryIO.OdometryInputs inputs) {
    val robotPose = poseEstimator.getEstimatedPosition();

    inputs.robotPose = robotPose;
    inputs.robotRotation = robotPose.getRotation();

    val speeds = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      speeds[i] = modules[i].getState();
    }

    inputs.robotSpeeds = KINEMATICS.toChassisSpeeds(speeds);
  }

  @Override
  public void updateWithTime(
      double timestampFPGA, @NotNull SwerveModulePosition[] modulePositions) {
    val twist = KINEMATICS.toTwist2d(lastModulePositions, modulePositions);

    Rotation2d gyroAngle = Rotation2d.fromRadians(rawGyroPosition.getRadians() + twist.dtheta);

    updateWithTime(timestampFPGA, gyroAngle, modulePositions);
  }

  @Override
  public void updateWithTime(
      double timestampFPGA,
      @NotNull Rotation2d gyroAngle,
      @NotNull SwerveModulePosition[] modulePositions) {
    poseEstimator.updateWithTime(timestampFPGA, gyroAngle, modulePositions);
  }
}
