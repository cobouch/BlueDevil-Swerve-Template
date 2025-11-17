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
package org.salemrobotics.frc;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.val;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.salemrobotics.frc.commands.DriveCommands;
import org.salemrobotics.frc.subsystems.drive.SwerveDrive;
import org.salemrobotics.frc.util.FieldConstants;
import org.salemrobotics.frc.util.FieldConstants.ReefSide;
import org.salemrobotics.frc.util.PoseSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  @SuppressWarnings("unused")
  private final LoggedPowerDistribution powerDistribution;

  // Subsystems
  private final SwerveDrive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations

        powerDistribution = LoggedPowerDistribution.getInstance(0, ModuleType.kRev);
        drive = SwerveDrive.ofCTRE();
      }
      case SIM -> {
        SimulatedArena.overrideInstance(new Arena2025Reefscape());
        SimulatedArena.overrideSimulationTimings(Seconds.of(Robot.defaultPeriodSecs), 6);

        powerDistribution = LoggedPowerDistribution.getInstance();

        // Sim robot, instantiate physics sim IO implementations
        drive = SwerveDrive.ofMapleSim();
        SimulatedArena.getInstance().resetFieldForAuto();
      }
        // Uses default here because javac can't tell that we cover each enum variant
      default -> {
        // Replayed robot, disable IO implementations
        powerDistribution = LoggedPowerDistribution.getInstance();
        drive = SwerveDrive.ofReplay();
      }
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Configure auto align
    controller
        .leftBumper()
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));
    controller
        .rightBumper()
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));
  }

  @Contract("_ -> new")
  private @NotNull Command joystickApproach(PoseSupplier poseSupplier) {
    // TODO: fully implement JoystickApproach
    return Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void simulationPeriodic() {
    val arenaInstance = SimulatedArena.getInstance();
    arenaInstance.simulationPeriodic();

    Logger.recordOutput("FieldSimulation/Algae", arenaInstance.getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", arenaInstance.getGamePiecesArrayByType("Coral"));
  }
}
