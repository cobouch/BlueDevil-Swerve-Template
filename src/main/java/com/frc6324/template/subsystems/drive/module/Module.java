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

import static com.frc6324.template.subsystems.drive.DrivetrainConstants.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.RequiredArgsConstructor;
import lombok.val;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class Module {
  private final String logKey;

  private final ModuleIO io;
  private final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

  private SwerveModuleState setpoint = new SwerveModuleState();
  private SwerveModulePosition[] odomPositions = new SwerveModulePosition[0];

  private final Alert driveMotorConnected;
  private final Alert steerMotorConnected;
  private final Alert encoderConnected;

  public Module(ModuleIO io, String moduleName) {
    this.io = io;
    logKey = "Drive/" + moduleName;

    driveMotorConnected =
        new Alert(moduleName + " module drive motor disconnected", Alert.AlertType.kError);
    steerMotorConnected =
        new Alert(moduleName + " module steer motor disconnected", Alert.AlertType.kError);
    encoderConnected =
        new Alert(moduleName + " encoder pod is disconnected", Alert.AlertType.kError);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
  }

  public void modulePeriodic() {
    odomPositions = new SwerveModulePosition[inputs.odometryDrivePositions.length];
    for (int i = 0; i < odomPositions.length; i++) {
      val drivePosition = inputs.odometryDrivePositions[i] * WHEEL_RADIUS.in(Meters);
      val steerAngle = inputs.odometrySteerPositions[i];

      odomPositions[i] = new SwerveModulePosition(drivePosition, steerAngle);
    }

    if (DriverStation.isDisabled()) {
      stop();
    }

    driveMotorConnected.set(!inputs.driveConnected);
    steerMotorConnected.set(!inputs.steerConnected);
    encoderConnected.set(!inputs.encoderConnected);
  }

  public SwerveModuleState runSetpoint(
      SwerveModuleState newSetpoint,
      LinearAcceleration wheelAccel,
      Force robotFeedForwardX,
      Force robotFeedForwardY,
      Force wheelFeedforward) {
    setpoint.optimize(getSteerFacing());
    val desiredMotorVelocity =
        RadiansPerSecond.of(
            (newSetpoint.speedMetersPerSecond / WHEEL_RADIUS.in(Meters)) * DRIVE_GEAR_RATIO);

    Translation2d forceVector =
        new Translation2d(robotFeedForwardX.in(Newtons), robotFeedForwardY.in(Newtons));

    double forceVectorMagnitude = forceVector.getNorm();
    double robotModuleFFForce;
    if (forceVectorMagnitude < 1e3) {
      robotModuleFFForce = 0;
    } else {
      robotModuleFFForce =
          forceVectorMagnitude * forceVector.getAngle().minus(getSteerFacing()).getCos();
    }
    double wheelFFTorque =
        (robotModuleFFForce + wheelFeedforward.in(Newtons)) * WHEEL_RADIUS.in(Meters);
    double motorFFTorque = wheelFFTorque / DRIVE_GEAR_RATIO;

    val angularAccel =
        RadiansPerSecondPerSecond.of(
            wheelAccel.in(MetersPerSecondPerSecond) / WHEEL_CIRCUMFERENCE.in(Meters));

    Logger.recordOutput(logKey + "/Setpoint/State", newSetpoint);
    Logger.recordOutput(logKey + "/Setpoint/DriveMotorVelocity", desiredMotorVelocity);
    Logger.recordOutput(logKey + "/Setpoint/SteerPosition", newSetpoint.angle);
    Logger.recordOutput(logKey + "/Setpoint/FeedforwardNewtonMeters", motorFFTorque);
    Logger.recordOutput(logKey + "/Setpoint/LinearAccelMPS", wheelAccel);
    Logger.recordOutput(logKey + "/Setpoint/AngularAccelRotPerSec", angularAccel);
    Logger.recordOutput(logKey + "/Setpoint/ForceVector", forceVector);

    io.requestState(
        desiredMotorVelocity, angularAccel, NewtonMeters.of(motorFFTorque), newSetpoint.angle);

    setpoint = newSetpoint;
    return setpoint;
  }

  public void stop() {
    val state = new SwerveModuleState(0, getSteerFacing());

    Logger.recordOutput(logKey + "/Setpoint/State", state);
    Logger.recordOutput(logKey + "/Setpoint/DriveMotorVelocity", 0.0);
    Logger.recordOutput(logKey + "/Setpoint/SteerPosition", getSteerFacing());
    Logger.recordOutput(logKey + "/Setpoint/FeedforwardNewtonMeters", 0.0);
    Logger.recordOutput(logKey + "/Setpoint/LinearAccelMPS", 0.0);
    Logger.recordOutput(logKey + "/Setpoint/AngularAccelRotPerSec", 0.0);
    Logger.recordOutput(logKey + "/Setpoint/AngularAccelRotPerSec", 0.0);
    Logger.recordOutput(logKey + "/Setpoint/ForceVector", Translation2d.kZero);

    io.brake();
    io.brake();
  }

  public Rotation2d getSteerFacing() {
    return inputs.steerPosition;
  }

  public AngularVelocity getSteerVelocity() {
    return inputs.steerVelocity;
  }

  public Distance getDrivePosition() {
    return Meters.of(inputs.drivePosition.in(Radians) * WHEEL_RADIUS.in(Meters));
  }

  public LinearVelocity getDriveVelocity() {
    return MetersPerSecond.of(inputs.driveVelocity.in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getSteerFacing());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerFacing());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odomPositions;
  }

  public SwerveModulePosition getOdometryPosition(int idx) {
    return odomPositions[idx];
  }

  public void runVoltageCharacterization(Rotation2d steerFacing, Voltage driveOutput) {
    io.setDriveOpenLoop(driveOutput);
    io.requestSteerPosition(steerFacing);
  }

  public void runCurrentCharacterization(Rotation2d steerFacing, Current driveOutput) {
    io.setDriveOpenLoop(driveOutput);
    io.requestSteerPosition(steerFacing);
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosition.in(Radians);
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocity.in(RotationsPerSecond);
  }
}
