package com.frc6324.template.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLog;

/**
 * An I/O layer over the robot's drivetrain. Defines I/O operations and methods to control the
 * drivetrain.
 */
public interface DriveIO {
  /** Inputs for the drivetrain. */
  @AutoLog
  public class DriveInputs extends SwerveDriveState {
    /** The angle of the robot as reported by the gyroscope. */
    public Rotation2d GyroAngle = Rotation2d.kZero;

    public double VelocityTimestamp = 0;

    public Angle Roll = Radians.zero();
    public Angle Pitch = Radians.zero();
    public AngularVelocity RollVelocity = RadiansPerSecond.zero();
    public AngularVelocity PitchVelocity = RadiansPerSecond.zero();
    public AngularVelocity YawVelocity = RadiansPerSecond.zero();
    public LinearAcceleration AccelerationX = MetersPerSecondPerSecond.zero();
    public LinearAcceleration AccelerationY = MetersPerSecondPerSecond.zero();

    public DriveInputs() {
      Pose = Pose2d.kZero;
      RawHeading = Rotation2d.kZero;
      Speeds = new ChassisSpeeds();
      Timestamp = 0;

      ModulePositions = new SwerveModulePosition[4];
      Arrays.fill(ModulePositions, new SwerveModulePosition());

      ModuleStates = new SwerveModuleState[4];
      ModuleTargets = new SwerveModuleState[4];

      var state = new SwerveModuleState();
      Arrays.fill(ModuleStates, state);
      Arrays.fill(ModuleTargets, state);
    }

    public void copyFromState(SwerveDriveState state) {
      Pose = state.Pose;
      SuccessfulDaqs = state.SuccessfulDaqs;
      FailedDaqs = state.FailedDaqs;
      ModuleStates = state.ModuleStates;
      ModulePositions = state.ModulePositions;
      Speeds = state.Speeds;
      OdometryPeriod = state.OdometryPeriod;
    }
  }

  /**
   * Updates the drivetrain's loggable inputs.
   *
   * @param inputs
   */
  void updateInputs(DriveInputs inputs);

  /**
   * Logs the module states.
   *
   * @param state The states to log.
   */
  void logModuleStates(SwerveDriveState state);

  /**
   * Resets the pose estimator's position.
   *
   * @param pose The pose to reset to.
   */
  void resetPose(Pose2d pose);

  /**
   * Commands the drivetrain to follow a specified request.
   *
   * @param request The request to apply.
   */
  void setControl(SwerveRequest request);

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param pose The measured pose.
   * @param timestamp The timestamp at which the measurement was taken by the camera.
   * @param stddevs The standard deviations of the measurement.
   */
  void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stddevs);

  /** An empty implementation of I/O procedures for the drivetrain during replay. */
  public static final class DriveIOReplay implements DriveIO {
    @Override
    public void updateInputs(DriveInputs inputs) {}

    @Override
    public void logModuleStates(SwerveDriveState state) {}

    @Override
    public void resetPose(Pose2d pose) {}

    @Override
    public void setControl(SwerveRequest request) {}

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stddevs) {}
  }
}
