package com.frc6324.template.subsystems.drive;

import static com.frc6324.template.subsystems.drive.DrivetrainConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import org.littletonrobotics.junction.Logger;

public sealed class DriveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DriveIO permits DriveIOSim {
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private final SwerveDriveState state;

  public DriveIOCTRE(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        drivetrainConstants,
        ODOMETRY_UPDATE_FREQUENCY,
        ODOMETRY_STDDEVS,
        DEFAULT_VISION_STDDEVS,
        modules);
    super.resetPose(STARTING_POSE);

    state = getState();
    Pigeon2 pigeon = getPigeon2();

    pitchVelocity = pigeon.getAngularVelocityYWorld();
    rollVelocity = pigeon.getAngularVelocityXWorld();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    roll = pigeon.getRoll();
    pitch = pigeon.getPitch();
    accelerationX = pigeon.getAccelerationX();
    accelerationY = pigeon.getAccelerationY();

    BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_UPDATE_FREQUENCY, yawVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, pitchVelocity, rollVelocity, roll, pitch, accelerationX, accelerationY);

    getOdometryThread().setThreadPriority(99);
  }

  @Override
  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.copyFromState(state);

    inputs.GyroAngle = getPigeon2().getRotation2d();

    BaseStatusSignal.refreshAll(
        pitchVelocity, rollVelocity, yawVelocity, roll, pitch, accelerationX, accelerationY);

    inputs.Roll = roll.getValue();
    inputs.Pitch = pitch.getValue();
    inputs.RollVelocity = rollVelocity.getValue();
    inputs.PitchVelocity = pitchVelocity.getValue();
    inputs.YawVelocity = yawVelocity.getValue();
    inputs.AccelerationX = accelerationX.getValue();
    inputs.AccelerationY = accelerationY.getValue();
  }

  @Override
  public void logModuleStates(SwerveDriveState state) {
    if (state.ModuleStates == null) {
      return;
    }

    final var modules = getModules();
    for (int i = 0; i < modules.length; i++) {
      var module = modules[i];
      var name = MODULE_NAMES[i];

      Logger.recordOutput(
          "Drive/" + name + "/Absolute Encoder Angle",
          module.getEncoder().getAbsolutePosition().getValue());
      Logger.recordOutput("Drive/" + name + "/Steering Angle", state.ModuleStates[i].angle);
      Logger.recordOutput("Drive/" + name + "/Target Steering Angle", state.ModuleTargets[i].angle);
      Logger.recordOutput(
          "Drive/" + name + "/Drive Velocity", state.ModuleStates[i].speedMetersPerSecond);
      Logger.recordOutput(
          "Drive/" + name + "/Target Drive Velocity", state.ModuleTargets[i].speedMetersPerSecond);
    }
  }

  @Override
  public void setVisionStdDevs(double stdX, double stdY, double stdTheta) {
    super.setVisionMeasurementStdDevs(VecBuilder.fill(stdX, stdY, stdTheta));
  }
}
