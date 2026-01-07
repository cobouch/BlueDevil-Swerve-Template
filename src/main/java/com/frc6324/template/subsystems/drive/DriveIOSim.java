package com.frc6324.template.subsystems.drive;

import static com.frc6324.template.subsystems.drive.DrivetrainConstants.*;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.frc6324.template.generated.TunerConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.Consumer;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * The {@code DriveIOSim} class extends {@link DriveIOHardware} to provide simulation-specific
 * functionality for the swerve drive system. It integrates with WPILib's simulation framework for
 * testing and development.
 */
public final class DriveIOSim extends DriveIOCTRE {
  public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

  Pose2d lastConsumedPose = null;
  Consumer<SwerveDriveState> simTelemetryConsumer =
      swerveDriveState ->
          swerveDriveState.Pose =
              mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();

  public DriveIOSim(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(driveTrainConstants, modules);

    // Rewrite the telemetry consumer with a consumer for sim
    registerTelemetry(simTelemetryConsumer);

    mapleSimSwerveDrivetrain =
        new MapleSimSwerveDrivetrain(
            ROBOT_MASS,
            Inches.of(30),
            Inches.of(30),
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            WHEEL_COF,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            modules);
  }

  public void resetPose(Pose2d pose) {
    mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
    super.resetPose(pose);
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    mapleSimSwerveDrivetrain.update();
    updateSimState(0.02, SimulatedBattery.getBatteryVoltage().in(Volts));

    super.updateInputs(inputs);
  }

  public MapleSimSwerveDrivetrain getMapleSimDrive() {
    return mapleSimSwerveDrivetrain;
  }
}
