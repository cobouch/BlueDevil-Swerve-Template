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

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import lombok.RequiredArgsConstructor;
import lombok.val;
import org.jetbrains.annotations.NotNull;

@RequiredArgsConstructor
public class OdometryIOCTRE implements OdometryIO {
  private final SwerveDrivetrain<?, ?, ?> drivetrain;

  @Override
  public void addVisionMeasurement(
      @NotNull Pose2d visionPose, double timestampFPGA, @NotNull Vector<N3> stddevs) {
    drivetrain.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestampFPGA), stddevs);
  }

  @Override
  public Optional<Pose2d> samplePoseAtTime(double timestampFPGA) {
    try {
      return drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(timestampFPGA));
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  @Override
  public void setPose(@NotNull Pose2d pose) {
    drivetrain.resetPose(pose);
  }

  @Override
  public void updateInputs(@NotNull OdometryIO.OdometryInputs inputs) {
    val state = drivetrain.getState();

    inputs.robotPose = state.Pose;
    inputs.robotRotation = state.Pose.getRotation();
    inputs.robotSpeeds = state.Speeds;
  }
}
