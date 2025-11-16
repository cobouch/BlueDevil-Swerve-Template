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
package org.salemrobotics.frc.subsystems.drive.module;

import static org.salemrobotics.frc.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import lombok.val;
import org.jetbrains.annotations.NotNull;
import org.salemrobotics.frc.generated.TunerConstants;
import org.salemrobotics.frc.subsystems.drive.DrivetrainConstants;

public class ModuleIOTalonFX extends ModuleIOTalonBase<TalonFX, CANcoder> {
  private final ClosedLoopOutputType driveOutputType;
  private final ClosedLoopOutputType steerOutputType;

  protected final VelocityVoltage driveRequest_Voltage = new VelocityVoltage(0);
  protected final VelocityTorqueCurrentFOC driveRequest_TorqueCurrent =
      new VelocityTorqueCurrentFOC(0);

  protected final PositionVoltage steerRequest_Voltage = new PositionVoltage(0);
  protected final MotionMagicExpoTorqueCurrentFOC steerRequest_TorqueCurrent =
      new MotionMagicExpoTorqueCurrentFOC(0);

  protected ModuleIOTalonFX(
      @NotNull ClosedLoopOutputType driveOutputType,
      @NotNull ClosedLoopOutputType steerOutputType,
      @NotNull TalonFX driveMotor,
      @NotNull TalonFX steerMotor,
      @NotNull CANcoder encoder) {
    super(false, driveMotor, steerMotor, encoder);

    this.driveOutputType = driveOutputType;
    this.steerOutputType = steerOutputType;
  }

  public ModuleIOTalonFX(
      @NotNull
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              constants) {
    super(
        true,
        new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus),
        new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus),
        new CANcoder(constants.EncoderId, TunerConstants.kCANBus));

    this.driveOutputType = constants.DriveMotorClosedLoopOutput;
    this.steerOutputType = constants.SteerMotorClosedLoopOutput;

    val driveConfig = constants.DriveMotorInitialConfigs;

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0, 0.25));

    val steerConfig = constants.SteerMotorInitialConfigs;

    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    steerConfig.Slot0 = constants.SteerMotorGains;
    steerConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    steerConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException("Unsupported swerve configuration");
        };
    steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicAcceleration =
        steerConfig.MotionMagic.MotionMagicCruiseVelocity * 7.5;
    steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

    tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerConfig, 0.25));
    tryUntilOk(5, () -> steerTalon.setPosition(0, 0.25));

    val cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfig, 0.25));
    tryUntilOk(5, () -> encoder.setPosition(0, 0.25));
  }

  protected final ControlRequest getDriveRequest(
      @NotNull AngularVelocity velocity,
      @NotNull AngularAcceleration acceleration,
      @NotNull Torque torqueFF) {
    return switch (driveOutputType) {
      case Voltage ->
          driveRequest_Voltage
              .withVelocity(velocity)
              .withAcceleration(acceleration)
              .withFeedForward(
                  DrivetrainConstants.calculateFFVoltage(torqueFF, driveVelocity.getValue()));
      case TorqueCurrentFOC ->
          driveRequest_TorqueCurrent
              .withVelocity(velocity)
              .withAcceleration(acceleration)
              .withFeedForward(DrivetrainConstants.calculateFFCurrent(torqueFF));
    };
  }

  protected final ControlRequest getSteerRequest(@NotNull Rotation2d heading) {
    val steerAngle = heading.getRotations();

    return switch (steerOutputType) {
      case Voltage -> steerRequest_Voltage.withPosition(steerAngle);
      case TorqueCurrentFOC -> steerRequest_TorqueCurrent.withPosition(steerAngle);
    };
  }

  @Override
  public void requestDriveVelocity(
      @NotNull AngularVelocity velocity,
      @NotNull AngularAcceleration acceleration,
      @NotNull Torque torqueFF) {
    driveTalon.setControl(getDriveRequest(velocity, acceleration, torqueFF));
  }

  @Override
  public void requestSteerPosition(@NotNull Rotation2d heading) {
    steerTalon.setControl(getSteerRequest(heading));
  }
}
