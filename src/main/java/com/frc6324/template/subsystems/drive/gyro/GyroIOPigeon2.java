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
package com.frc6324.template.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.frc6324.template.generated.TunerConstants;
import com.frc6324.template.subsystems.drive.odometry.AKOdometryThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;
import lombok.val;
import org.jetbrains.annotations.NotNull;

/**
 * An I/O abstraction over a CTR Electronics Pigeon 2.
 *
 * @see Pigeon2
 * @see GyroIO
 */
public final class GyroIOPigeon2 implements GyroIO {
  /** The implementation of the Pigeon 2. */
  private final Pigeon2 pigeon;

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final BlockingDeque<Rotation2d> odometryYawPositions = new LinkedBlockingDeque<>(25);

  /**
   * Constructs a new I/O implementation of the drivetrain's Pigeon, using the constants found in
   * {@link TunerConstants#DrivetrainConstants}.
   *
   * @see TunerConstants
   */
  public GyroIOPigeon2() {
    this(new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus));

    AKOdometryThread.addTask(
        () -> {
          if (odometryYawPositions.remainingCapacity() == 0) {
            odometryYawPositions.removeFirst();
          }

          val yawPosition = BaseStatusSignal.getLatencyCompensatedValue(yaw, yawVelocity);
          odometryYawPositions.addLast(Rotation2d.fromRadians(yawPosition.in(Radians)));
        },
        yaw,
        yawVelocity);
  }

  /**
   * Constructs a new I/O implementation of a Pigeon 2.
   *
   * @param pigeon The pigeon being wrapped.
   */
  public GyroIOPigeon2(@NotNull Pigeon2 pigeon) {
    this.pigeon = pigeon;

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
  }

  /**
   * Constructs a new I/O implementation of the provided drivetrain's Pigeon.
   *
   * @param drivetrain The Swerve API drivetrain which holds the pigeon.
   */
  public GyroIOPigeon2(@NotNull SwerveDrivetrain<?, ?, ?> drivetrain) {
    this(drivetrain.getPigeon2());
  }

  @Override
  public void updateInputs(@NotNull GyroInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.yawPosition = pigeon.getRotation2d();
    inputs.yawVelocityRadPerSec = yawVelocity.getValue();
    inputs.odometryYawPositions = odometryYawPositions.toArray(Rotation2d[]::new);
  }
}
