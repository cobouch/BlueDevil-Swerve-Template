package com.frc6324.template.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.frc6324.template.Robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

/**
 *
 *
 * <h2>Injects Maple-Sim simulation data into a CTRE swerve drivetrain.</h2>
 *
 * <p>This class retrieves simulation data from Maple-Sim and injects it into the CTRE {@link
 * com.ctre.phoenix6.swerve.SwerveDrivetrain} instance.
 *
 * <p>It replaces the {@link com.ctre.phoenix6.swerve.SimSwerveDrivetrain} class.
 */
public class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  private final SimSwerveModule[] simModules;
  public final SwerveDriveSimulation mapleSimDrive;

  /**
   *
   *
   * <h2>Constructs a drivetrain simulation using the specified parameters.</h2>
   *
   * @param simPeriod the time period of the simulation
   * @param robotMassWithBumpers the total mass of the robot, including bumpers
   * @param bumperLengthX the length of the bumper along the X-axis (influences the collision space
   *     of the robot)
   * @param bumperWidthY the width of the bumper along the Y-axis (influences the collision space of
   *     the robot)
   * @param driveMotorModel the {@link DCMotor} model for the drive motor, typically <code>
   *     DCMotor.getKrakenX60Foc()
   *     </code>
   * @param steerMotorModel the {@link DCMotor} model for the steer motor, typically <code>
   *     DCMotor.getKrakenX60Foc()
   *     </code>
   * @param wheelCOF the coefficient of friction of the drive wheels
   * @param moduleLocations the locations of the swerve modules on the robot, in the order <code>
   *     FL, FR, BL, BR</code>
   * @param pigeon the {@link Pigeon2} IMU used in the drivetrain
   * @param modules the {@link SwerveModule}s, typically obtained via {@link
   *     SwerveDrivetrain#getModules()}
   * @param moduleConstants the constants for the swerve modules
   */
  @SuppressWarnings("unchecked")
  public MapleSimSwerveDrivetrain(Mass robotMassWithBumpers, Distance bumperLengthX, Distance bumperWidthY, DCMotor driveMotorModel, DCMotor steerMotorModel, double wheelCOF, Translation2d[] moduleLocations, Pigeon2 pigeon, SwerveModule<TalonFX, TalonFX, CANcoder>[] modules, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    this.pigeonSim = pigeon.getSimState();
    simModules = new SimSwerveModule[moduleConstants.length];
    DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLengthX, bumperWidthY)
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(COTS.ofMark4i(DCMotor.getKrakenX60Foc(1), DCMotor.getKrakenX60Foc(1), wheelCOF, 2));
    mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d(3, 3, Rotation2d.kZero));

    SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
    for (int i = 0; i < this.simModules.length; i++) {
      var constants =
          (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
              (moduleConstants[0]);
      simModules[i] = new SimSwerveModule(constants, moduleSimulations[i], modules[i]);
    }

    SimulatedArena.overrideSimulationTimings(Seconds.of(Robot.defaultPeriodSecs), 5);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /**
   *
   *
   * <h2>Update the simulation.</h2>
   *
   * <p>Updates the Maple-Sim simulation and injects the results into the simulated CTRE devices,
   * including motors and the IMU.
   */
  public void update() {
    var instance = SimulatedArena.getInstance();
    instance.simulationPeriodic();
    Logger.recordOutput("FieldSimulation/Algae", instance.getGamePiecesArrayByType("algae"));
    Logger.recordOutput("FieldSimulation/Coral", instance.getGamePiecesArrayByType("coral"));

    pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(
        RadiansPerSecond.of(
            mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                .omegaRadiansPerSecond));
  }

  /**
   *
   *
   * <h1>Represents the simulation of a single {@link SwerveModule}.</h1>
   */
  protected static class SimSwerveModule {
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        moduleConstant;
    public final SwerveModuleSimulation moduleSimulation;

    public SimSwerveModule(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            moduleConstant,
        SwerveModuleSimulation moduleSimulation,
        SwerveModule<TalonFX, TalonFX, CANcoder> module) {
      this.moduleConstant = moduleConstant;
      this.moduleSimulation = moduleSimulation;
      moduleSimulation.useDriveMotorController(
          new TalonFXMotorControllerSim(module.getDriveMotor()));
      moduleSimulation.useSteerMotorController(
          new TalonFXMotorControllerWithRemoteCanCoderSim(
              module.getSteerMotor(), module.getEncoder()));
    }
  }

  // Static utils classes
  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = talonFX.getDeviceID();
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCanCoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }
}
