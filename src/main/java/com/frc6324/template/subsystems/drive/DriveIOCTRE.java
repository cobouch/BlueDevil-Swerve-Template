package com.frc6324.template.subsystems.drive;

import static com.frc6324.template.subsystems.drive.DrivetrainConstants.DEFAULT_VISION_STDDEVS;
import static com.frc6324.template.subsystems.drive.DrivetrainConstants.MODULE_NAMES;
import static com.frc6324.template.subsystems.drive.DrivetrainConstants.ODOMETRY_STDDEVS;
import static com.frc6324.template.subsystems.drive.DrivetrainConstants.ODOMETRY_UPDATE_FREQUENCY;
import static com.frc6324.template.subsystems.drive.DrivetrainConstants.STARTING_POSE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.frc6324.template.generated.TunerConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class DriveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
  private final StatusSignal<AngularVelocity> pitchVelocitySignal;
  private final StatusSignal<AngularVelocity> rollVelocitySignal;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> rollSignal;
  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private final SwerveDriveState state;

  public DriveIOCTRE() {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        TunerConstants.DrivetrainConstants,
        ODOMETRY_UPDATE_FREQUENCY,
        ODOMETRY_STDDEVS,
        DEFAULT_VISION_STDDEVS,
        SwerveDrive.regulateModuleConstantsForSimulation(
            new SwerveModuleConstants[] {
              TunerConstants.FrontLeft,
              TunerConstants.FrontRight,
              TunerConstants.BackLeft,
              TunerConstants.BackRight
            }));

    // Reset the pose in simulation to prevent CTRE/maple-sim conflict(s)
    if (RobotBase.isSimulation()) {
      super.resetPose(STARTING_POSE);
    }

    // Get the state and pigeon of the drivetrain
    state = getState();
    Pigeon2 pigeon = getPigeon2();

    // Store signals from the pigeon we care about
    pitchVelocitySignal = pigeon.getAngularVelocityYWorld();
    rollVelocitySignal = pigeon.getAngularVelocityXWorld();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    rollSignal = pigeon.getRoll();
    pitchSignal = pigeon.getPitch();
    accelerationX = pigeon.getAccelerationX();
    accelerationY = pigeon.getAccelerationY();

    // Set the update frequencies for the gyro signals
    BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_UPDATE_FREQUENCY, yawVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        pitchVelocitySignal,
        rollVelocitySignal,
        rollSignal,
        pitchSignal,
        accelerationX,
        accelerationY);

    // Set the odometry thread's priority to very high
    getOdometryThread().setThreadPriority(99);
  }

  @Override
  public void logModuleStates(SwerveDriveState state) {
    // Stop if the module states or targets are null so we don't cause an NPE
    if (state.ModuleStates == null || state.ModuleTargets == null) {
      return;
    }

    for (int i = 0; i < 4; i++) {
      // Get the current module and its name
      var module = getModule(i);
      var name = MODULE_NAMES[i];

      // Log steering information
      Logger.recordOutput(
          "Drive/" + name + "/Absolute Encoder Angle",
          module.getEncoder().getAbsolutePosition().getValue());
      Logger.recordOutput("Drive/" + name + "/Steering Angle", state.ModuleStates[i].angle);
      Logger.recordOutput("Drive/" + name + "/Target Steering Angle", state.ModuleTargets[i].angle);
      // Log drive velocity information
      Logger.recordOutput(
          "Drive/" + name + "/Drive Velocity", state.ModuleStates[i].speedMetersPerSecond);
      Logger.recordOutput(
          "Drive/" + name + "/Target Drive Velocity", state.ModuleTargets[i].speedMetersPerSecond);
    }
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    // Copy the recorded state into the inputs
    inputs.copyFromState(state);

    // Rip the gyro angle straight from the pigeon
    inputs.GyroAngle = getPigeon2().getRotation2d();

    // Refresh all of the status signals
    BaseStatusSignal.refreshAll(
        pitchVelocitySignal,
        rollVelocitySignal,
        yawVelocity,
        rollSignal,
        pitchSignal,
        accelerationX,
        accelerationY);

    // Record states we don't care about for the safety below
    inputs.YawVelocity = yawVelocity.getValue();

    // Get all of the other gyro values we care about
    var roll = rollSignal.getValue();
    var pitch = pitchSignal.getValue();
    var rollVelocity = rollVelocitySignal.getValue();
    var pitchVelocity = pitchVelocitySignal.getValue();
    var accelX = accelerationX.getValue();
    var accelY = accelerationY.getValue();

    // Send all of the tilt values to the driving safety util
    DrivingSafety.update(roll, pitch, rollVelocity, pitchVelocity, accelX, accelY);

    // Finally, write all of the tils values into the inputs.
    inputs.Roll = roll;
    inputs.RollVelocity = rollVelocity;
    inputs.Pitch = pitch;
    inputs.PitchVelocity = pitchVelocity;
    inputs.AccelerationX = accelX;
    inputs.AccelerationY = accelY;
  }
}
