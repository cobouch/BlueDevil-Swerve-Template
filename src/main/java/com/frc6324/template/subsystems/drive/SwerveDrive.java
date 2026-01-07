package com.frc6324.template.subsystems.drive;

import static com.frc6324.template.subsystems.drive.DrivetrainConstants.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.frc6324.template.generated.TunerConstants;
import com.frc6324.template.util.AllianceFlipUtil;
import com.frc6324.template.util.LocalADStarAK;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class SwerveDrive extends SubsystemBase {
  DriveIO io;
  DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();

  private final ApplyRobotSpeeds autoRequest =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withDesaturateWheelSpeeds(true);

  public SwerveDrive(DriveIO io) {
    this.io = io;

    configurePathPlanner();
  }

  private void configurePathPlanner() {
    ModuleConfig modulecfg =
        new ModuleConfig(
            Meters.of(TunerConstants.FrontLeft.WheelRadius),
            TunerConstants.kSpeedAt12Volts,
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1),
            Amps.of(TunerConstants.FrontLeft.SlipCurrent),
            1);

    RobotConfig config = new RobotConfig(ROBOT_MASS, ROBOT_MOI, modulecfg, MODULE_TRANSLATIONS);

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (speeds, ffs) -> io.setControl(autoRequest.withSpeeds(speeds)),
        new PPHolonomicDriveController(new PIDConstants(WHEEL_COF), new PIDConstants(WHEEL_COF)),
        config,
        AllianceFlipUtil::shouldFlip,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        path -> {
          Logger.recordOutput("Odometry/Trajectory", path.toArray(Pose2d[]::new));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", pose);
        });

    SmartDashboard.putData(new SwerveWidget(this));
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    io.logModuleStates(inputs);

    Logger.recordOutput("Drive/Update Latency", Timer.getFPGATimestamp() - timestamp);
  }

  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  public Pose2d getPose() {
    return inputs.Pose;
  }

  public void setPose(Pose2d pose) {
    io.resetPose(pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return inputs.Speeds;
  }

  public SwerveDriveState getState() {
    return inputs;
  }

  public LinearVelocity getMaxLinearSpeed() {
    return TunerConstants.kSpeedAt12Volts;
  }

  public AngularVelocity getMaxAngularSpeed() {
    return RadiansPerSecond.of(getMaxLinearSpeed().in(MetersPerSecond) / DRIVE_BASE_RADIUS);
  }

  public static final SwerveModuleConstants<?, ?, ?>[] MODULE_CONSTANTS = {
    TunerConstants.FrontLeft,
    TunerConstants.FrontRight,
    TunerConstants.BackLeft,
    TunerConstants.BackRight
  };

  /**
   *
   *
   * <h2>Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.</h2>
   *
   * <p>This method processes an array of {@link SwerveModuleConstants} to apply necessary
   * adjustments for simulation purposes, ensuring compatibility and avoiding known bugs.
   *
   * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
   */
  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
      regulateModuleConstantForSimulation(moduleConstant);
    }

    return moduleConstants;
  }

  /**
   *
   *
   * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
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
   * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants
   * used on real robot hardware.</h4>
   */
  private static void regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) {
      return;
    }

    // Apply simulation-specific adjustments to module constants
    moduleConstants
        // Disable encoder offsets
        .withEncoderOffset(0)
        // Disable motor inversions for drive and steer motors
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        // Disable CANcoder inversion
        .withEncoderInverted(false)
        // Adjust steer motor PID gains
        .withSteerMotorGains(moduleConstants.SteerMotorGains.withKP(70).withKD(4.5))
        // Adjust friction voltages
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.15))
        // Adjust steer inertia
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}
