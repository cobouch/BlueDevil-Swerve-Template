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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.jetbrains.annotations.NotNull;
import org.salemrobotics.frc.generated.TunerConstants;
import org.salemrobotics.frc.util.Statics;

public final class DrivetrainConstants {
  public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(300);
  public static final Frequency CTRE_ODOM_FREQUENCY = Hertz.of(500);

  public static final Vector<N3> AK_ODOMETRY_STDDEVS = VecBuilder.fill(0.08, 0.08, 0.08);
  public static final Vector<N3> CTRE_ODOMETRY_STDDEVS = VecBuilder.fill(0.04, 0.04, 0.04);

  public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1)
    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio);
  public static final DCMotor STEER_MOTOR = DCMotor.getKrakenX60Foc(1)
    .withReduction(TunerConstants.FrontLeft.SteerMotorGearRatio);

  public static final Mass ROBOT_MASS = Pounds.of(140);
  public static final double WHEEL_COF = 1.43;
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);

  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  };
  public static final String[] MODULE_NAMES = {
    "Front Left", "Front Right", "Back Left", "Back Right"
  };

  public static final Distance WHEEL_RADIUS = Meters.of(TunerConstants.FrontLeft.WheelRadius);
  public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2 * Math.PI);
  public static final LinearVelocity MAX_DRIVE_VELOCITY = MetersPerSecond.of(4.73);

  public static final double DRIVE_GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(MODULE_TRANSLATIONS[0].getNorm(), MODULE_TRANSLATIONS[1].getNorm()),
          Math.max(MODULE_TRANSLATIONS[2].getNorm(), MODULE_TRANSLATIONS[3].getNorm()));

  public static final PIDConstants AUTO_TRANSLATION_CONSTANTS = new PIDConstants(5, 0, 0);
  public static final PIDConstants AUTO_ROTATION_CONSTANTS = new PIDConstants(7, 0, 0);

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  public static final RobotConfig PATH_PLANNER_CONFIG =
      Statics.initOrDefault(
          RobotConfig::fromGUISettings,
          () ->
              new RobotConfig(
                  ROBOT_MASS,
                  ROBOT_MOI,
                  new ModuleConfig(
                      WHEEL_RADIUS,
                      MAX_DRIVE_VELOCITY,
                      WHEEL_COF,
                      DRIVE_MOTOR,
                      Amps.of(TunerConstants.FrontLeft.SlipCurrent),
                      1),
                  MODULE_TRANSLATIONS));

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(ROBOT_MASS)
          .withBumperSize(Inches.of(30), Inches.of(30))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(COTS.ofMark4i(
              DCMotor.getKrakenX60Foc(1),
              DCMotor.getKrakenX60Foc(1), 
              WHEEL_COF, 
              2))
          .withCustomModuleTranslations(MODULE_TRANSLATIONS);

  public static @NotNull Voltage calculateFFVoltage(@NotNull Torque torque, @NotNull AngularVelocity driveVelocity) {
    return Volts.of(DRIVE_MOTOR.getVoltage(torque.in(NewtonMeters), driveVelocity.in(RadiansPerSecond)));
  }

  public static @NotNull Current calculateFFCurrent(@NotNull Torque torque) {
    return Amps.of(DRIVE_MOTOR.getCurrent(torque.in(NewtonMeters)));
  }
}
