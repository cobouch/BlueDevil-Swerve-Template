package com.frc6324.template.subsystems.drive;

import static com.frc6324.template.subsystems.drive.DrivetrainConstants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.frc6324.template.generated.TunerConstants;
import com.frc6324.template.util.AllianceFlipUtil;
import com.frc6324.template.util.LocalADStarAK;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.FollowPath.Builder;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Represents the drivetrain subsystem, used for moving the robot. */
public final class SwerveDrive extends SubsystemBase {
  private final DriveIO io;
  private final DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();
  private final Builder blineBuilder;
  private final ApplyRobotSpeeds autoRequest =
      new ApplyRobotSpeeds()
          .withDesaturateWheelSpeeds(true)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);
  private final SwerveDriveBrake brakeRequest = new SwerveDriveBrake();

  /**
   * Creates a new drivetrain subsystem.
   *
   * @param io The implementation of the I/O the drivetrain uses.
   */
  public SwerveDrive(DriveIO io) {
    setName("Drivetrain");

    this.io = io;

    // Create the builder that BLine will use
    blineBuilder =
        new Builder(
                this,
                this::getPose,
                this::getChassisSpeeds,
                speeds -> io.setControl(autoRequest.withSpeeds(speeds)),
                BLINE_TRANSLATION_CONTROLLER,
                BLINE_ROTATION_CONTROLLER,
                BLINE_CTE_CONTROLLER)
            .withDefaultShouldFlip()
            .withPoseReset(this::setPoseIfSim);

    // Configure logging callbacks for
    FollowPath.setPoseLoggingConsumer(
        (data) -> {
          Logger.recordOutput("BLine/" + data.getFirst(), data.getSecond());
        });
    FollowPath.setBooleanLoggingConsumer(
        (data) -> {
          Logger.recordOutput("BLine/" + data.getFirst(), data.getSecond());
        });
    FollowPath.setDoubleLoggingConsumer(
        (data) -> {
          Logger.recordOutput("BLine/" + data.getFirst(), data.getSecond());
        });
    FollowPath.setTranslationListLoggingConsumer(
        (data) -> {
          Logger.recordOutput("BLine/" + data.getFirst(), data.getSecond());
        });

    // Configure PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPoseIfSim,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> {
          io.setControl(
              autoRequest
                  .withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY()));
        },
        new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
        PP_CONFIG,
        AllianceFlipUtil::shouldFlip,
        this);

    // Configure pathfinding for AdvantageKit
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Configure PathPlanner logging
    PathPlannerLogging.setLogActivePathCallback(
        (path) -> {
          Logger.recordOutput("PathPlanner/ActiveTrajectory", path.toArray(Pose2d[]::new));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (target) -> {
          Logger.recordOutput("PathPlanner/Target", target);
        });

    // Put the swerve drive widget to SmartDashboard
    SmartDashboard.putData(new SwerveWidget(this));
  }

  /**
   * Commands the drivetrain to follow a specified swerve control request.
   *
   * @param request The request to apply.
   * @return The drive command.
   */
  public Command applyRequest(SwerveRequest request) {
    return run(() -> setControl(request));
  }

  /**
   * Adds a vision measurement to the drivetrain's pose estimator.
   *
   * @param pose The estimated robot pose.
   * @param timestamp The timestamp of the observation.
   * @param stddevs The standard deviation values of the measurement.
   */
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stddevs) {
    io.addVisionMeasurement(pose, timestamp, stddevs);
  }

  /**
   * Commands the drivetrain to move into an X position for maximum traction.
   *
   * @return The braking command.
   */
  public Command brake() {
    return run(() -> applyRequest(brakeRequest));
  }

  /**
   * Gets the BLine builder configured for the drivetrain.
   *
   * @return The builder used to build BLine paths.
   */
  public Builder getBLineBuilder() {
    return blineBuilder;
  }

  /**
   * Gets the speed of the robot.
   *
   * @return The speeds being experienced by the drivetrain chassis.
   */
  @AutoLogOutput(key = "Odometry/MeasuredSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return inputs.Speeds;
  }

  /**
   * Gets the robot's current estimated pose.
   *
   * @return The pose of the robot.
   */
  @AutoLogOutput(key = "Odometry/RobotPose")
  public Pose2d getPose() {
    return inputs.Pose;
  }

  /**
   * Gets the current state of the drivetrain.
   *
   * @return The drivetrain's state.
   */
  SwerveDriveState getState() {
    return inputs;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    io.logModuleStates(inputs);

    Logger.recordOutput("Drive/UpdateLatency", Timer.getFPGATimestamp() - timestamp);
  }

  /**
   * Sets the active swerve control request of the drivetrain.
   *
   * @param request The new swerve request to run.
   */
  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  /**
   * Sets the robot's odometry pose unconditionally.
   *
   * @param pose The pose to reset to.
   */
  public void setPose(Pose2d pose) {
    io.resetPose(pose);
  }

  /**
   * Sets the robot's pose if the robot profram is running in a simulator.
   *
   * @param pose The pose to reset to.
   */
  public void setPoseIfSim(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      setPose(pose);
    }
  }

  public static double getMaxAngularSpeed() {
    return getMaxLinearSpeed() / DRIVE_BASE_RADIUS;
  }

  public static AngularVelocity getMaxAngularSpeedMeasure() {
    return RadiansPerSecond.of(getMaxAngularSpeed());
  }

  public static double getMaxLinearSpeed() {
    return getMaxLinearSpeedMeasure().in(MetersPerSecond);
  }

  public static LinearVelocity getMaxLinearSpeedMeasure() {
    var voltage =
        RobotBase.isSimulation()
            ? SimulatedBattery.getBatteryVoltage().in(Volts)
            : RobotController.getBatteryVoltage();
    return TunerConstants.kSpeedAt12Volts.times(voltage / 12);
  }

  /**
   * The set of constants for each swerve module.
   *
   * <p>These are UNREGULATED - you must manually call {@link
   * #regulateModuleConstantsForSimulation(SwerveModuleConstants[])} on this array to do so.
   */
  public static final SwerveModuleConstants<?, ?, ?>[] MODULE_CONSTANTS = {
    TunerConstants.FrontLeft,
    TunerConstants.FrontRight,
    TunerConstants.BackLeft,
    TunerConstants.BackRight
  };

  public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;
  public static final SteerRequestType STEER_REQUEST_TYPE = SteerRequestType.Position;

  /**
   * Regulates a set of {@link SwerveModuleConstants} for drivetrain simulation.
   *
   * <p>This method processes an array of {@link SwerveModuleConstants} to apply necessary
   * adjustments for simulation purposes, ensuring compatibility and avoiding known bugs.
   *
   * @param moduleConstants The set of constants to modify.
   * @return The modified constants. This is guaranteed to be equivalent to {@code moduleConstants}.
   * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
   */
  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    // Regulate each module's constants
    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
      regulateModuleConstantForSimulation(moduleConstant);
    }

    // Return the constants for easier use.
    return moduleConstants;
  }

  /**
   * Regulates a set of {@link SwerveModuleConstants} for a single module.
   *
   * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation
   * purposes. These changes have no effect on real robot operations and address known simulation
   * bugs:
   *
   * <ul>
   *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted
   *       configurations.
   *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization
   *       issues.
   *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve
   *       simulation performance.
   * </ul>
   *
   * @param moduleConstants The constants to modify during simulation.
   * @implNote This function is a no-op when running on a real robot. If this function is called on
   *     a real robot, the inputted constants will not be modified.
   */
  private static void regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) {
      return;
    }

    // Disable encoder offsets and inversion
    moduleConstants.EncoderOffset = 0;
    moduleConstants.EncoderInverted = false;

    // Disable motor inversions
    moduleConstants.DriveMotorInverted = false;
    moduleConstants.SteerMotorInverted = false;

    // Use Voltage output during sim since Torque Current doesn't play nice
    moduleConstants.SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
    moduleConstants.SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // Alter motor PID gains for both sim and voltage
    moduleConstants.DriveMotorGains.kP = 5;
    moduleConstants.DriveMotorGains.kD = 0;
    moduleConstants.DriveMotorGains.kS = 0;

    moduleConstants.SteerMotorGains.kP = 80;
    moduleConstants.SteerMotorGains.kD = 8;
    moduleConstants.SteerMotorGains.kS = 0.6;
    moduleConstants.SteerMotorGains.kV = 0;

    // Adjust friction voltages for sim
    moduleConstants.DriveFrictionVoltage = 0.1;
    moduleConstants.SteerFrictionVoltage = 0.15;

    // Adjust steer inertia
    moduleConstants.SteerInertia = 0.05;
  }

  /**
   * An extension of PathPlanner's {@link PPHolonomicDriveController} that logs the output of the
   * controller.
   */
  public class SwerveDriveControllerAK extends PPHolonomicDriveController {
    /** Constructs a new holonomic drive controller. */
    public SwerveDriveControllerAK() {
      super(PATHPLANNER_TRANSLATION_CONSTANTS, PATHPLANNER_ROTATION_CONSTANTS);
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(
        Pose2d currentPose, PathPlannerTrajectoryState targetState) {
      var speeds = super.calculateRobotRelativeSpeeds(currentPose, targetState);

      Logger.recordOutput("PathPlanner/ControllerOutput", speeds);
      return speeds;
    }
  }
}
