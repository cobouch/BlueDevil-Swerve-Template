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
package com.frc6324.template.util;

import com.frc6324.template.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LoggedProfiledPID extends ProfiledPIDController {
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber iZone;
  private final LoggedTunableNumber maxVelocity;
  private final LoggedTunableNumber maxAcceleration;

  /**
   * @param tableKey The name of this PID controller.
   */
  public LoggedProfiledPID(
      String tableKey, double p, double i, double d, double maxVelocity, double maxAcceleration) {
    super(p, i, d, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    kP = new LoggedTunableNumber(tableKey + "/kP", p);
    kI = new LoggedTunableNumber(tableKey + "/kI", i);
    kD = new LoggedTunableNumber(tableKey + "/kD", d);
    iZone = new LoggedTunableNumber(tableKey + "/IZone", 0);
    this.maxVelocity = new LoggedTunableNumber(tableKey + "/MaxVelocity", maxVelocity);
    this.maxAcceleration = new LoggedTunableNumber(tableKey + "/MaxAcceleration", maxAcceleration);
  }

  public void updatePID() {
    if (!Constants.DEVBOT) {
      // skip if not devbot, since there is no dashboard number to edit
      return;
    }

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      setPID(kP.get(), kI.get(), kD.get());
    }

    if (iZone.hasChanged()) {
      setIZone(iZone.get());
    }

    if (maxVelocity.hasChanged() || maxAcceleration.hasChanged()) {
      setConstraints(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }
  }
}
