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

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Torque;
import org.jetbrains.annotations.NotNull;

public class ModuleIOCTRETalonFX extends ModuleIOTalonFX {
  private final SwerveModule<TalonFX, TalonFX, CANcoder> module;

  public ModuleIOCTRETalonFX(@NotNull SwerveModule<TalonFX, TalonFX, CANcoder> module) {
    super(
        module.getDriveClosedLoopOutputType(),
        module.getSteerClosedLoopOutputType(),
        module.getDriveMotor(),
        module.getSteerMotor(),
        module.getEncoder());

    this.module = module;
  }

  @Override
  public void brake() {
    module.apply(brakeRequest, brakeRequest);
  }

  @Override
  public void coast() {
    module.apply(coastRequest, coastRequest);
  }

  @Override
  public void requestState(
      @NotNull AngularVelocity velocity,
      @NotNull AngularAcceleration acceleration,
      @NotNull Torque torqueFF,
      @NotNull Rotation2d heading) {
    module.apply(getDriveRequest(velocity, acceleration, torqueFF), getSteerRequest(heading));
  }
}
