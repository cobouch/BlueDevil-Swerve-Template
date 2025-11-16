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
package org.salemrobotics.frc.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Newtons;
import static org.salemrobotics.frc.subsystems.drive.DrivetrainConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Consumer;
import lombok.val;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.salemrobotics.frc.generated.TunerConstants;
import org.salemrobotics.frc.subsystems.drive.gyro.GyroIO;
import org.salemrobotics.frc.subsystems.drive.gyro.GyroIOMapleSim;
import org.salemrobotics.frc.subsystems.drive.gyro.GyroIOPigeon2;
import org.salemrobotics.frc.subsystems.drive.gyro.GyroInputsAutoLogged;
import org.salemrobotics.frc.subsystems.drive.module.*;
import org.salemrobotics.frc.subsystems.drive.module.Module;
import org.salemrobotics.frc.subsystems.drive.odometry.*;
import org.salemrobotics.frc.util.AllianceFlipUtil;
import org.salemrobotics.frc.util.DeltaTimeCalculator;
import org.salemrobotics.frc.util.IOLayer;
import org.salemrobotics.frc.util.LocalADStarAK;

public class SwerveDrive extends SubsystemBase {
  @FunctionalInterface
  public interface ModuleConsumer {
    void accept(int idx, Module module);
  }

  private static final DriveFeedforwards INITIAL_FEEDFORWARDS = DriveFeedforwards.zeros(4);

  private final CANBusIO canbus;
  private final CANBusInputsAutoLogged canbusInputs = new CANBusInputsAutoLogged();
  private final Alert canBusHighUtilization =
      new Alert("High utilization of the drivetrain CAN Bus!", Alert.AlertType.kError);

  private final GyroIO gyro;
  private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
  private final Alert gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.kError);

  private final OdometryIO odometry;
  private final OdometryInputsAutoLogged odometryInputs = new OdometryInputsAutoLogged();

  private final Module[] modules = new Module[4];

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(PATH_PLANNER_CONFIG, DegreesPerSecond.of(540));
  private SwerveSetpoint previousSetpoint;
  private final DeltaTimeCalculator deltaTime = new DeltaTimeCalculator();

  public SwerveDrive(
      CANBusIO canbus,
      GyroIO gyro,
      ModuleIO fl,
      ModuleIO fr,
      ModuleIO bl,
      ModuleIO br,
      OdometryIO odometry) {
    this.canbus = canbus;
    this.gyro = gyro;
    modules[0] = new Module(fl, "Front Left");
    modules[1] = new Module(fr, "Front Right");
    modules[2] = new Module(bl, "Back Left");
    modules[3] = new Module(br, "Back Right");
    this.odometry = odometry;
    odometry.setModules(modules);

    AKOdometryThread.start();

    gyroDisconnected.set(false);
    canBusHighUtilization.set(false);

    AutoBuilder.configure(
        this::getPose,
        this::setPoseIfSim,
        this::getChassisSpeeds,
        this::runVelocityWithFeedforward,
        new PPHolonomicDriveController(AUTO_TRANSLATION_CONSTANTS, AUTO_ROTATION_CONSTANTS),
        PATH_PLANNER_CONFIG,
        AllianceFlipUtil::shouldFlip,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) ->
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(Pose2d[]::new)));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    previousSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), INITIAL_FEEDFORWARDS);
  }

  public static SwerveDrive ofAKit() {
    return new SwerveDrive(
        new CANBusIOReal(),
        new GyroIOPigeon2(),
        new ModuleIOTalonFX(TunerConstants.FrontLeft),
        new ModuleIOTalonFX(TunerConstants.FrontRight),
        new ModuleIOTalonFX(TunerConstants.BackLeft),
        new ModuleIOTalonFX(TunerConstants.BackRight),
        new OdometryIOAK());
  }

  public static SwerveDrive ofCTRE() {
    val drivetrain =
        new SwerveDrivetrain<>(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            TunerConstants.DrivetrainConstants,
            CTRE_ODOM_FREQUENCY.in(Hertz),
            CTRE_ODOMETRY_STDDEVS,
            VecBuilder.fill(0, 0, 0),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

    return new SwerveDrive(
        new CANBusIOReal(),
        new GyroIOPigeon2(drivetrain),
        new ModuleIOCTRETalonFX(drivetrain.getModule(0)),
        new ModuleIOCTRETalonFX(drivetrain.getModule(1)),
        new ModuleIOCTRETalonFX(drivetrain.getModule(2)),
        new ModuleIOCTRETalonFX(drivetrain.getModule(3)),
        new OdometryIOCTRE(drivetrain));
  }

  public static SwerveDrive ofMapleSim() {
    val simulation =
        new SwerveDriveSimulation(MAPLE_SIM_CONFIG, new Pose2d(3, 3, Rotation2d.kZero));
    SimulatedArena.getInstance().addDriveTrainSimulation(simulation);
    val modules = simulation.getModules();

    return new SwerveDrive(
        inputs -> {},
        new GyroIOMapleSim(simulation.getGyroSimulation()),
        new ModuleIOMapleSim(modules[0]),
        new ModuleIOMapleSim(modules[1]),
        new ModuleIOMapleSim(modules[2]),
        new ModuleIOMapleSim(modules[3]),
        new OdometryIOMapleSim(simulation));
  }

  public static SwerveDrive ofReplay() {
    return new SwerveDrive(
        IOLayer::replay,
        IOLayer::replay,
        IOLayer::replay,
        IOLayer::replay,
        IOLayer::replay,
        IOLayer::replay,
        IOLayer::replay);
  }

  @Override
  public void periodic() {
    // Obtain the global odometry lock and update all the drivetrain I/O's inputs
    try {
      AKOdometryThread.GLOBAL_LOCK.lock();

      gyro.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      canbus.updateInputs(canbusInputs);
      Logger.processInputs("Drive/CANBus", canbusInputs);

      forEachModule(Module::updateInputs);
      for (val module : modules) {
        module.updateInputs();
      }

      odometry.updateInputs(odometryInputs);
      Logger.processInputs("Drive", odometryInputs);
      Logger.recordOutput("Drive/OdometryInitialPose", odometryInputs.robotPose);
    } finally {
      AKOdometryThread.GLOBAL_LOCK.unlock();
    }

    forEachModule(Module::modulePeriodic);

    if (DriverStation.isDisabled()) {
      // Stop moving when disabled (for redundancy)
      stop();

      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[0]);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[0]);
    }

    val timestamps = odometry.getOdometryTimestamps();
    val gyroConnected = gyroInputs.connected;
    for (int i = 0; i < timestamps.length; i++) {
      val sample = i;

      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

      forEachModule(
          (moduleIdx, module) -> modulePositions[moduleIdx] = module.getOdometryPosition(sample));

      val timestamp = timestamps[sample];
      if (gyroConnected) {
        odometry.updateWithTime(
            timestamp, gyroInputs.odometryYawPositions[sample], modulePositions);
      } else {
        odometry.updateWithTime(timestamp, modulePositions);
      }
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    Voltage batteryVoltage;
    if (RobotBase.isSimulation()) {
      batteryVoltage = SimulatedBattery.getBatteryVoltage();
    } else {
      batteryVoltage = RobotController.getMeasureBatteryVoltage();
    }

    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, speeds, deltaTime.getMeasure(), batteryVoltage);

    val states = previousSetpoint.moduleStates();
    val feedforwards = previousSetpoint.feedforwards();

    val xForces = feedforwards.robotRelativeForcesX();
    val yForces = feedforwards.robotRelativeForcesY();
    val wheelForces = feedforwards.linearForces();
    val accelerations = feedforwards.accelerations();

    Logger.recordOutput("SwerveStates/Setpoints", states);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", previousSetpoint.robotRelativeSpeeds());

    forEachModule(
        (idx, module) ->
            module.runSetpoint(
                states[idx], accelerations[idx], xForces[idx], yForces[idx], wheelForces[idx]));

    Logger.recordOutput("SwerveStates/Optimized", states);
  }

  public void runVelocityWithFeedforward(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    previousSetpoint =
        new SwerveSetpoint(
            previousSetpoint.robotRelativeSpeeds(), previousSetpoint.moduleStates(), feedforwards);

    runVelocity(speeds);
  }

  public void runFieldCentric(ChassisSpeeds speeds) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation()));
  }

  public void runAllianceCentric(ChassisSpeeds speeds) {
    var rotation = getRotation();
    if (AllianceFlipUtil.shouldFlip()) {
      rotation = rotation.plus(Rotation2d.k180deg);
    }

    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation));
  }

  public void stop() {
    forEachModule(Module::stop);
  }

  public void stopWithX() {
    val headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = MODULE_TRANSLATIONS[i].getAngle();
    }
    KINEMATICS.resetHeadings(headings);

    val states = KINEMATICS.toSwerveModuleStates(new ChassisSpeeds());
    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), states, INITIAL_FEEDFORWARDS);

    forEachModule(
        (idx, module) ->
            module.runSetpoint(
                states[idx],
                MetersPerSecondPerSecond.zero(),
                Newtons.zero(),
                Newtons.zero(),
                Newtons.zero()));
  }

  public void forEachModule(ModuleConsumer action) {
    for (int i = 0; i < 4; i++) {
      action.accept(i, modules[i]);
    }
  }

  public void forEachModule(Consumer<Module> action) {
    for (int i = 0; i < 4; i++) {
      action.accept(modules[i]);
    }
  }

  public void runVoltageCharacterization(Voltage output) {
    forEachModule(module -> module.runVoltageCharacterization(Rotation2d.kZero, output));
  }

  public void runCurrentCharacterization(Current output) {
    forEachModule(module -> module.runCurrentCharacterization(Rotation2d.kZero, output));
  }

  /** Returns the states (facing and drive velocity) of each swerve module. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    val states = new SwerveModuleState[4];
    forEachModule((idx, module) -> states[idx] = module.getState());
    return states;
  }

  /** Returns the positions (facing and drive position) of each module. */
  public SwerveModulePosition[] getModulePositions() {
    val positions = new SwerveModulePosition[4];
    forEachModule((idx, module) -> positions[idx] = module.getPosition());
    return positions;
  }

  /** Returns the measured chassis speeds of the robot. */
  public ChassisSpeeds getChassisSpeeds() {
    return odometryInputs.robotSpeeds;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    forEachModule((i, module) -> values[i] = module.getWheelRadiusCharacterizationPosition());

    return values;
  }

  /** Returns the average velocity of the modules in rotations/second (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity();
    }
    return output / 4;
  }

  /**
   * Return the robot's current pose.
   *
   * @return The pose measured by the robot's odometry.
   */
  public Pose2d getPose() {
    return odometryInputs.robotPose;
  }

  public Optional<Pose2d> getPoseAtTime(double timestamp) {
    return odometry.samplePoseAtTime(timestamp);
  }

  public Rotation2d getRotation() {
    return odometryInputs.robotRotation;
  }

  public void setPose(Pose2d pose) {
    odometry.setPose(pose);
  }

  public void setPoseIfSim(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      setPose(pose);
    }
  }

  public LinearVelocity getMaxLinearSpeed() {
    return TunerConstants.kSpeedAt12Volts;
  }

  public AngularVelocity getMaxAngularSpeed() {
    return RadiansPerSecond.of(getMaxLinearSpeed().in(MetersPerSecond) / DRIVE_BASE_RADIUS);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> stddevs) {
    odometry.addVisionMeasurement(pose, timestamp, stddevs);
  }
}
