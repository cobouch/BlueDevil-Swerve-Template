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
package com.frc6324.template.subsystems.drive;

import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLog;

/** An I/O abstraction over a CAN bus, allowing us to log the state of the drivetrain's CAN bus. */
@FunctionalInterface
public interface CANBusIO {
  /** Inputs for a CAN bus I/O layer. Allows for logging of CAN bus stats. */
  @AutoLog
  class CANBusInputs {
    /** The name of the CAN bus (e.g. "rio", "canivore0"). */
    public String name = "None";

    /** The Utilization% the CAN bus is currently experiencing. */
    public double utilization = 0;

    /** The CAN bus' current off count. */
    public int offCount = 0;

    /** The CAN bus' transmit buffer full count. */
    public int txFullCount = 0;

    /** The amount of receive errors the bus has experienced. */
    public int receiveErrorCounter = 0;

    /** The amount of transmit errors the bus has experienced. */
    public int transmitErrorCounter = 0;
  }

  /**
   * Updates the inputs for this CAN bus.
   *
   * @param inputs The inputs to modify.
   */
  void updateInputs(@NotNull CANBusInputs inputs);
}
