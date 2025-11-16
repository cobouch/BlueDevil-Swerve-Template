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
package org.salemrobotics.frc.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import lombok.val;
import org.ironmaple.simulation.SimulatedArena;
import org.jetbrains.annotations.NotNull;
import org.salemrobotics.frc.Robot;

/** A utility class for using Phoenix 6 devices. */
public class PhoenixUtil {
  /**
   * Attempts to run a command until no error is produced.
   *
   * @param maxAttempts The maximum number of times the command can be run before this function
   *     gives up.
   * @param command The command to run.
   * @return Whether the command succeeded.
   * @see StatusCode
   * @see #tryUntilOk(int, Supplier, String)
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    StatusCode error = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < maxAttempts; i++) {
      // Run the command
      error = command.get();

      // Stop if the status is OK
      if (error.isOK()) {
        return true;
      }
    }

    // Report the error message to the DriverStation
    DriverStation.reportError("Error running phoenix command: " + error, true);
    // Default to returning false (since the error wasn't ever OK)
    return false;
  }

  /**
   * Attempts to run a command until no error is produced.
   *
   * @param maxAttempts The maximum number of times the command can be run before this function
   *     gives up.
   * @param command The command to run.
   * @param errMessage A message to print if the command does not succeed.
   * @return Whether the command succeeded.
   * @see StatusCode
   */
  public static boolean tryUntilOk(
      int maxAttempts, Supplier<StatusCode> command, String errMessage) {
    StatusCode error = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < maxAttempts; i++) {
      // Run the command
      error = command.get();

      // Stop if the status is OK
      if (error.isOK()) {
        return true;
      }
    }

    DriverStation.reportError("Error running phoenix command: " + error, false);
    // Report the error with the provided message
    DriverStation.reportError(errMessage, true);
    return false;
  }

  /**
   * Finds the average timestamp of a group of Phoenix 6 signals relative to Phoenix 6's current
   * time.
   *
   * @param signals The signals to take into account.
   * @return The average timestamp of each of the signals.
   * @implNote Because Phoenix 6 uses its own timebase, the timestamp returned is relative to the
   *     'current' timebase. The current time can be found by using {@link
   *     Utils#getCurrentTimeSeconds()}. If you plan on using this to feed a {@link
   *     edu.wpi.first.math.estimator.PoseEstimator PoseEstimator} class, ensure that you convert
   *     this timestamp into FPGA time (which vision systems normally use) by using {@link
   *     #currentToFPGATime(double)}.
   * @see com.ctre.phoenix6.StatusSignal
   * @see BaseStatusSignal
   * @see #currentToFPGATime(double)
   */
  public static double averageTimestamp(BaseStatusSignal @NotNull ... signals) {
    // Prevent division by zero
    if (signals.length == 0) return 0;

    double averageTimestamp = 0;
    for (val signal : signals) {
      averageTimestamp += signal.getTimestamp().getTime();
    }

    return averageTimestamp / signals.length;
  }

  /**
   * Finds the average timestamp of a group of Phoenix 6 signals in FPGA time.
   *
   * @param signals The signals to take into account.
   * @return The average timestamp of each of the signals.
   * @implNote This is functionally equivalent to {@code
   *     currentToFPGATime(averageTimestamp(signals)) }
   * @see com.ctre.phoenix6.StatusSignal
   * @see BaseStatusSignal
   * @see #currentToFPGATime(double)
   */
  public static double averageTimestampFPGA(BaseStatusSignal @NotNull ... signals) {
    return currentToFPGATime(averageTimestamp(signals));
  }

  /**
   * Converts a timestamp from Phoenix 6's current timebase to the RoboRIO's FPGA timebase.
   *
   * @param currentTime The timestamp to convert in seconds.
   * @return The timestamp in seconds relative to FPGA time.
   */
  public static double currentToFPGATime(double currentTime) {
    // Current to FPGA time is calculated via:
    // (getCurrentTimeSeconds() - Timer.getFPGATimestamp()) + fpgaTimeSeconds;
    // So by flipping the offset we can go backwards.
    return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + currentTime;
  }

  /**
   * Creates a list of simulated timestamps for simulation updates. This is used to create
   * timestamps corresponding to maple-sim's updates as it does not do so itself.
   *
   * @return The timestamps of the simulation updates.
   */
  public static double @NotNull [] getSimulationOdometryTimestamps() {
    val odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp()
              - Robot.defaultPeriodSecs
              + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }
}
