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
package com.frc6324.template;

import com.frc6324.template.commands.DriveCommands;
import com.frc6324.template.subsystems.drive.DriveIO.DriveIOReplay;
import com.frc6324.template.subsystems.drive.DriveIOCTRE;
import com.frc6324.template.subsystems.drive.DriveIOSim;
import com.frc6324.template.subsystems.drive.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDrive drive;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        drive = new SwerveDrive(new DriveIOCTRE());
      }
      case SIM -> {
        drive = new SwerveDrive(new DriveIOSim());
      }
      default -> {
        drive = new SwerveDrive(new DriveIOReplay());
      }
    }

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
