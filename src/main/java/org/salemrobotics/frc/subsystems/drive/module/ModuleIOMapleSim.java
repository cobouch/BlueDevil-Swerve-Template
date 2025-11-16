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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import java.util.Arrays;
import lombok.val;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.jetbrains.annotations.NotNull;
import org.salemrobotics.frc.generated.TunerConstants;
import org.salemrobotics.frc.subsystems.drive.DrivetrainConstants;

public final class ModuleIOMapleSim implements ModuleIO {
  private static final double DRIVE_KP = 0.12;
  private static final double DRIVE_KD = 0;
  private static final double STEER_KP = 2;
  private static final double STEER_KD = 0;

  private final SwerveModuleSimulation simulation;
  private final GenericMotorController driveMotor;
  private final GenericMotorController steerMotor;

  private boolean driveClosedLoop = false;
  private boolean steerClosedLoop = false;

  private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private final PIDController steerController = new PIDController(STEER_KP, 0, STEER_KD);

  private Voltage driveAppliedVolts = Volts.zero();
  private Voltage steerAppliedVolts = Volts.zero();
  private AngularVelocity desiredMotorVelocity = RadiansPerSecond.zero();
  private Rotation2d desiredSteerAngle = Rotation2d.kZero;
  private Voltage torqueFFVolts = Volts.zero();

  public ModuleIOMapleSim(SwerveModuleSimulation sim) {
    simulation = sim;
    driveMotor =
        simulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(TunerConstants.FrontLeft.SlipCurrent));
    steerMotor = simulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(60));

    // Enable turn wrapping for PID
    steerController.enableContinuousInput(-Math.PI, Math.PI);
    SimulatedArena.getInstance().addCustomSimulation(subtick -> runControlLoops());
  }

  @Override
  public void updateInputs(@NotNull ModuleInputs inputs) {
    inputs.driveConnected = true;
    inputs.steerConnected = true;
    inputs.encoderConnected = true;

    inputs.drivePosition = simulation.getDriveWheelFinalPosition();
    inputs.driveVelocity = simulation.getDriveWheelFinalSpeed();
    inputs.driveAppliedVoltage = simulation.getDriveMotorAppliedVoltage();
    inputs.driveStatorCurrent = simulation.getDriveMotorStatorCurrent();
    inputs.driveTorqueCurrent = Amps.zero();

    inputs.steerPosition = simulation.getSteerAbsoluteFacing();
    inputs.steerVelocity = simulation.getSteerAbsoluteEncoderSpeed();
    inputs.steerAbsolutePosition = inputs.steerPosition;
    inputs.steerAbsoluteVelocity = inputs.steerVelocity;
    inputs.steerAppliedVoltage = simulation.getSteerMotorAppliedVoltage();
    inputs.steerStatorCurrent = simulation.getSteerMotorStatorCurrent();
    inputs.steerTorqueCurrent = Amps.zero();

    inputs.odometryDrivePositions =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(a -> a.in(Radians))
            .toArray();
    inputs.odometrySteerPositions = simulation.getCachedSteerAbsolutePositions();
  }

  private void runControlLoops() {
    if (driveClosedLoop) {
      calculateDriveOutput();
    }
    if (steerClosedLoop) {
      calculateSteerOutput();
    }

    driveMotor.requestVoltage(driveAppliedVolts);
    steerMotor.requestVoltage(steerAppliedVolts);
  }

  private void calculateDriveOutput() {
    val motorModel = simulation.getDriveMotorConfigs().motor;
    double desiredSpeed = desiredMotorVelocity.in(RadiansPerSecond);

    if (!Double.isFinite(desiredSpeed) || Math.abs(desiredSpeed) < 1e-6) {
      driveAppliedVolts = Volts.zero();
      return;
    }

    double frictionTorque =
        motorModel.getTorque(
            motorModel.getCurrent(0, TunerConstants.FrontLeft.DriveFrictionVoltage));
    frictionTorque *= Math.signum(desiredSpeed);

    double velocityFF =
        motorModel.getVoltage(frictionTorque, desiredMotorVelocity.in(RadiansPerSecond));

    val feedforwardVolts = Volts.of(velocityFF).plus(torqueFFVolts);
    double feedbackVolts =
        driveController.calculate(
            simulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
            desiredMotorVelocity.in(RadiansPerSecond));

    driveAppliedVolts = feedforwardVolts.plus(Volts.of(feedbackVolts));
  }

  private void calculateSteerOutput() {
    double measured = simulation.getSteerAbsoluteAngle().in(Radians);
    if (!Double.isFinite(measured) || Math.abs(measured) > 1e6) {
      steerAppliedVolts = Volts.zero();
      return;
    }

    double desired = desiredSteerAngle.getRadians();
    double output = steerController.calculate(measured, desired);
    output = MathUtil.clamp(output, -12, 12);

    steerAppliedVolts = Volts.of(output);
  }

  @Override
  public void coast() {
    driveAppliedVolts = Volts.zero();
    driveClosedLoop = false;
    steerAppliedVolts = Volts.zero();
    steerClosedLoop = false;
  }

  @Override
  public void brake() {
    coast();
  }

  @Override
  public void setDriveOpenLoop(@NotNull Voltage output) {
    driveAppliedVolts = output;
    torqueFFVolts = Volts.zero();
    driveClosedLoop = false;
  }

  @Override
  public void setDriveOpenLoop(@NotNull Current output) {
    val motorModel = simulation.getDriveMotorConfigs().motor;

    driveAppliedVolts =
        Volts.of(
            motorModel.getVoltage(
                motorModel.getTorque(output.in(Amps)),
                simulation.getDriveEncoderUnGearedSpeed().in(RadiansPerSecond)));
    torqueFFVolts = Volts.zero();
    driveClosedLoop = false;
  }

  @Override
  public void setSteerOpenLoop(@NotNull Voltage output) {
    steerAppliedVolts = output;
    steerClosedLoop = false;
  }

  @Override
  public void setSteerOpenLoop(@NotNull Current output) {
    val motorModel = simulation.getSteerMotorConfigs().motor;

    steerAppliedVolts =
        Volts.of(
            motorModel.getVoltage(
                motorModel.getTorque(output.in(Amps)),
                simulation
                    .getSteerAbsoluteEncoderSpeed()
                    .div(TunerConstants.FrontLeft.SteerMotorGearRatio)
                    .in(RadiansPerSecond)));
  }

  @Override
  public void requestDriveVelocity(
      @NotNull AngularVelocity velocity,
      @NotNull AngularAcceleration acceleration,
      @NotNull Torque torqueFF) {
    if (!driveClosedLoop) {
      driveController.reset();
    }

    desiredMotorVelocity = velocity;
    driveClosedLoop = true;
    this.torqueFFVolts =
        DrivetrainConstants.calculateFFVoltage(torqueFF, simulation.getDriveEncoderUnGearedSpeed());
  }

  @Override
  public void requestSteerPosition(@NotNull Rotation2d heading) {
    if (!steerClosedLoop) {
      steerController.reset();
    }

    desiredSteerAngle = heading;
    steerClosedLoop = true;
  }
}
