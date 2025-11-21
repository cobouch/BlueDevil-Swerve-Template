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

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.frc6324.template.subsystems.drive.DrivetrainConstants;
import com.frc6324.template.subsystems.drive.odometry.AKOdometryThread;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;
import lombok.val;
import org.jetbrains.annotations.NotNull;

public abstract class ModuleIOTalonBase<M extends CommonTalon, E extends ParentDevice>
    implements ModuleIO {
  /** The drive motor's motor controller. */
  protected final M driveTalon;

  /** The steer motor's motor controller. */
  protected final M steerTalon;

  /** The azimuth encoder. */
  protected final E encoder;

  protected final StatusSignal<Angle> drivePosition;
  protected final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<AngularAcceleration> driveAcceleration;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;

  protected final StatusSignal<Angle> steerPosition;
  protected final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Voltage> steerAppliedVolts;
  private final StatusSignal<Current> steerStatorCurrent;
  private final StatusSignal<Current> steerTorqueCurrent;

  private final StatusSignal<Angle> steerAbsolutePosition;
  private final StatusSignal<AngularVelocity> steerAbsoluteVelocity;

  private final BlockingDeque<Angle> odometryDrivePositions = new LinkedBlockingDeque<>(25);
  private final BlockingDeque<Rotation2d> odometrySteerPositions = new LinkedBlockingDeque<>(25);

  protected final StaticBrake brakeRequest = new StaticBrake();
  protected final CoastOut coastRequest = new CoastOut();

  protected final VoltageOut openLoop_Voltage = new VoltageOut(0);
  protected final TorqueCurrentFOC openLoop_TorqueCurrent = new TorqueCurrentFOC(0);

  private final Debouncer driveConnected = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer steerConnected = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer encoderConnected = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Creates a new generic implementation of a swerve module using CTRE hardware
   *
   * @param addToAKThread Whether to add this module to the {@link AKOdometryThread}. Should only be
   *     used in conjunction with {@link
   *     org.salemrobotics.frc.subsystems.drive.odometry.OdometryIOAK OdometryIOAK}.
   * @param driveMotor The module's drive motor.
   * @param steerMotor The module's steer motor.
   * @param encoder The module's azimuth encoder.
   * @param encoderSource The source of the encoder. This is used to differentiate the different
   *     types of CANdi inputs, such as PWM1, PWM2 and Quadrature. This does not configure the steer
   *     motor to use the provided encoder as a feedback source.
   */
  protected ModuleIOTalonBase(
      boolean addToAKThread,
      @NotNull M driveMotor,
      @NotNull M steerMotor,
      @NotNull E encoder,
      @NotNull SwerveModuleConstants.SteerFeedbackType encoderSource) {
    driveTalon = driveMotor;
    steerTalon = steerMotor;
    this.encoder = encoder;

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAcceleration = driveTalon.getAcceleration();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveStatorCurrent = driveTalon.getStatorCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    steerPosition = steerTalon.getPosition();
    steerVelocity = steerTalon.getVelocity();
    steerAppliedVolts = steerTalon.getMotorVoltage();
    steerStatorCurrent = steerTalon.getStatorCurrent();
    steerTorqueCurrent = steerTalon.getTorqueCurrent();

    switch (encoderSource) {
      case RemoteCANcoder:
      case FusedCANcoder:
      case SyncCANcoder:
        if (encoder instanceof CANcoder cancoder) {
          steerAbsolutePosition = cancoder.getAbsolutePosition();
          steerAbsoluteVelocity = cancoder.getVelocity();
        } else {
          throw new RuntimeException(
              "Encoder feedback source does not match hardware object: Expected CANcoder, found "
                  + encoder.getClass().getSimpleName());
        }
        break;
      case RemoteCANdiPWM1:
      case FusedCANdiPWM1:
      case SyncCANdiPWM1:
        if (encoder instanceof CANdi candi) {
          steerAbsolutePosition = candi.getPWM1Position();
          steerAbsoluteVelocity = candi.getPWM1Velocity();
        } else {
          throw new RuntimeException(
              "Encoder feedback source does not match hardware object: Expected CANdi, found "
                  + encoder.getClass().getSimpleName());
        }
        break;
      case RemoteCANdiPWM2:
      case FusedCANdiPWM2:
      case SyncCANdiPWM2:
        if (encoder instanceof CANdi candi) {
          steerAbsolutePosition = candi.getPWM2Position();
          steerAbsoluteVelocity = candi.getPWM2Velocity();
        } else {
          throw new RuntimeException(
              "Encoder feedback source does not match hardware object: Expected CANdi, found "
                  + encoder.getClass().getSimpleName());
        }
        break;
      default:
        throw new RuntimeException(
            "Type of encoder '"
                + encoderSource
                + "' is not supported by the default ModuleIOTalonBase implementation. You must"
                + " manually add it yourself.");
    }

    if (addToAKThread) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          DrivetrainConstants.ODOMETRY_FREQUENCY,
          drivePosition,
          driveVelocity,
          steerPosition,
          steerVelocity);
      BaseStatusSignal.setUpdateFrequencyForAll(
          100,
          driveAcceleration,
          driveAppliedVolts,
          driveStatorCurrent,
          driveTorqueCurrent,
          steerAppliedVolts,
          steerStatorCurrent,
          steerTorqueCurrent,
          steerAbsolutePosition,
          steerAbsoluteVelocity);
      ParentDevice.optimizeBusUtilizationForAll(
          (ParentDevice) driveTalon, (ParentDevice) steerTalon, encoder);

      AKOdometryThread.addTask(
          () -> {
            if (odometryDrivePositions.remainingCapacity() == 0) {
              odometryDrivePositions.removeFirst();
              odometrySteerPositions.removeFirst();
            }

            val driveAngle =
                BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
            val steerAngle =
                BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);

            odometryDrivePositions.addLast((Angle) driveAngle);
            odometrySteerPositions.addLast(Rotation2d.fromRadians(steerAngle.in(Radians)));
          },
          drivePosition,
          driveVelocity,
          steerPosition,
          steerVelocity);
    }
  }

  @Override
  public void brake() {
    driveTalon.setControl(brakeRequest);
    steerTalon.setControl(brakeRequest);
  }

  @Override
  public void coast() {
    driveTalon.setControl(coastRequest);
    steerTalon.setControl(coastRequest);
  }

  @Override
  public void setDriveOpenLoop(@NotNull Current output) {
    driveTalon.setControl(openLoop_TorqueCurrent.withOutput(output));
  }

  @Override
  public void setDriveOpenLoop(@NotNull Voltage output) {
    driveTalon.setControl(openLoop_Voltage.withOutput(output));
  }

  @Override
  public void setSteerOpenLoop(@NotNull Current output) {
    steerTalon.setControl(openLoop_TorqueCurrent.withOutput(output));
  }

  @Override
  public void setSteerOpenLoop(@NotNull Voltage output) {
    steerTalon.setControl(openLoop_Voltage.withOutput(output));
  }

  @Override
  public void updateInputs(@NotNull ModuleInputs inputs) {
    val driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAcceleration,
            driveAppliedVolts,
            driveStatorCurrent,
            driveTorqueCurrent);
    val steerStatus =
        BaseStatusSignal.refreshAll(
            steerPosition,
            steerVelocity,
            steerAppliedVolts,
            steerStatorCurrent,
            steerTorqueCurrent);
    val encoderStatus = BaseStatusSignal.refreshAll(steerAbsolutePosition, steerAbsoluteVelocity);

    inputs.driveConnected = driveConnected.calculate(driveStatus.isOK());
    inputs.steerConnected = steerConnected.calculate(steerStatus.isOK());
    inputs.encoderConnected = encoderConnected.calculate(encoderStatus.isOK());

    val compensatedDrivePosition =
        BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    inputs.drivePosition = (Angle) compensatedDrivePosition;
    val compensatedDriveVelocity =
        BaseStatusSignal.getLatencyCompensatedValue(driveVelocity, driveAcceleration);
    inputs.driveVelocity = (AngularVelocity) compensatedDriveVelocity;
    inputs.driveAppliedVoltage = driveAppliedVolts.getValue();
    inputs.driveStatorCurrent = driveStatorCurrent.getValue();
    inputs.driveTorqueCurrent = driveTorqueCurrent.getValue();

    val compensatedSteerPosition =
        BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);
    inputs.steerPosition = new Rotation2d((Angle) compensatedSteerPosition);
    inputs.steerVelocity = steerVelocity.getValue();
    inputs.steerAppliedVoltage = steerAppliedVolts.getValue();
    inputs.steerStatorCurrent = steerStatorCurrent.getValue();
    inputs.steerTorqueCurrent = steerTorqueCurrent.getValue();

    inputs.odometryDrivePositions =
        odometryDrivePositions.stream().mapToDouble(a -> a.in(Radians)).toArray();
    inputs.odometrySteerPositions = odometrySteerPositions.toArray(Rotation2d[]::new);
  }
}
