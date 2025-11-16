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
package org.salemrobotics.frc.subsystems.drive.odometry;

import static org.salemrobotics.frc.subsystems.drive.DrivetrainConstants.KINEMATICS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import lombok.val;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.salemrobotics.frc.util.PhoenixUtil;

public class OdometryIOMapleSim implements OdometryIO {
  private final SwerveDriveSimulation simulation;
  private final TimeInterpolatableBuffer<Pose2d> robotPoses =
      TimeInterpolatableBuffer.createBuffer(1.5);
  private final SwerveModulePosition[] lastModulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final Rotation2d rawGyroAngle = Rotation2d.kZero;
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          KINEMATICS, rawGyroAngle, lastModulePositions, new Pose2d(3, 3, Rotation2d.kZero));

  @Contract(pure = true)
  public OdometryIOMapleSim(@NotNull SwerveDriveSimulation sim) {
    simulation = sim;
  }

  @Override
  public double[] getOdometryTimestamps() {
    return PhoenixUtil.getSimulationOdometryTimestamps();
  }

  @Override
  public Optional<Pose2d> samplePoseAtTime(double timestampFPGA) {
    try {
      return robotPoses.getSample(timestampFPGA);
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  @Override
  public void setPose(@NotNull Pose2d pose) {
    simulation.setSimulationWorldPose(pose);
  }

  @Override
  public void updateInputs(@NotNull OdometryIO.OdometryInputs inputs) {
    val robotPose = simulation.getSimulatedDriveTrainPose();

    robotPoses.addSample(Timer.getFPGATimestamp(), robotPose);
    inputs.robotPose = robotPose;
    inputs.robotRotation = robotPose.getRotation();
    inputs.robotSpeeds = simulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  @Override
  public void updateWithTime(
      double timestampFPGA, @NotNull SwerveModulePosition[] modulePositions) {
    val gyroDelta = KINEMATICS.toTwist2d(lastModulePositions, modulePositions).dtheta;
    updateWithTime(timestampFPGA, rawGyroAngle.plus(new Rotation2d(gyroDelta)), modulePositions);
  }

  @Override
  public void updateWithTime(
      double timestampFPGA,
      @NotNull Rotation2d gyroAngle,
      @NotNull SwerveModulePosition[] modulePositions) {
    poseEstimator.updateWithTime(timestampFPGA, gyroAngle, modulePositions);
  }
}
