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

import com.ctre.phoenix6.CANBus;
import lombok.val;
import org.jetbrains.annotations.NotNull;
import org.salemrobotics.frc.generated.TunerConstants;

/**
 * An I/O abstraction over a real CAN bus.
 *
 * @param bus The CAN bus being wrapped.
 * @see CANBus
 * @see CANBusIO
 */
public record CANBusIOReal(CANBus bus) implements CANBusIO {
  /**
   * Creates a new I/O abstraction over the drivetrain's CAN bus. This uses the CAN bus constant in
   * {@link TunerConstants}.
   *
   * @see TunerConstants
   * @see CANBus
   */
  public CANBusIOReal() {
    this(TunerConstants.kCANBus);
  }

  @Override
  public void updateInputs(@NotNull CANBusInputs inputs) {
    inputs.name = bus.getName();

    val status = bus.getStatus();

    inputs.utilization = status.BusUtilization;
    inputs.offCount = status.BusOffCount;
    inputs.txFullCount = status.TxFullCount;
    inputs.receiveErrorCounter = status.REC;
    inputs.transmitErrorCounter = status.TEC;
  }
}
