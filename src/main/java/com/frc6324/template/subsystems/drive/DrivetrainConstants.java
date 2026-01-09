package com.frc6324.template.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.frc6324.template.generated.TunerConstants;
import com.frc6324.template.util.Statics;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public final class DrivetrainConstants {
  public static final double ODOMETRY_UPDATE_FREQUENCY = 250;
  public static final int SIMULATION_TICKS_PER_LOOP = (int) (ODOMETRY_UPDATE_FREQUENCY / 50);

  public static final Vector<N3> ODOMETRY_STDDEVS =
      VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(2.5));
  public static final Vector<N3> DEFAULT_VISION_STDDEVS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  public static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  };

  public static final Mass ROBOT_MASS = Pounds.of(140);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6);
  public static final double WHEEL_COF = 1.43;

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(ROBOT_MASS)
          .withBumperSize(Inches.of(30), Inches.of(30))
          .withCustomModuleTranslations(MODULE_TRANSLATIONS)
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMark4i(DCMotor.getKrakenX60Foc(1), DCMotor.getKrakenX60Foc(1), WHEEL_COF, 2));

  public static final Pose2d STARTING_POSE = new Pose2d(3, 3, Rotation2d.kZero);

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public static final RobotConfig PP_CONFIG =
      Statics.initOrDefault(
          RobotConfig::fromGUISettings,
          () ->
              new RobotConfig(
                  ROBOT_MASS,
                  ROBOT_MOI,
                  new ModuleConfig(
                      Meters.of(TunerConstants.FrontLeft.WheelRadius),
                      TunerConstants.kSpeedAt12Volts,
                      WHEEL_COF,
                      DCMotor.getKrakenX60Foc(1),
                      Amps.of(TunerConstants.FrontLeft.SlipCurrent),
                      1),
                  MODULE_TRANSLATIONS));

  public static final PIDController BLINE_TRANSLATION_CONTROLLER =
      new PIDController(SIMULATION_TICKS_PER_LOOP, ODOMETRY_UPDATE_FREQUENCY, DRIVE_BASE_RADIUS);
  public static final PIDController BLINE_ROTATION_CONTROLLER =
      new PIDController(SIMULATION_TICKS_PER_LOOP, ODOMETRY_UPDATE_FREQUENCY, DRIVE_BASE_RADIUS);
  public static final PIDController BLINE_CTE_CONTROLLER =
      new PIDController(SIMULATION_TICKS_PER_LOOP, ODOMETRY_UPDATE_FREQUENCY, DRIVE_BASE_RADIUS);
  public static final PIDConstants PATHPLANNER_TRANSLATION_CONSTANTS = new PIDConstants(5, 0, 0);
  public static final PIDConstants PATHPLANNER_ROTATION_CONSTANTS = new PIDConstants(6, 0, 0);
}
