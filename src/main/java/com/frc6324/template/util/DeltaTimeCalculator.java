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

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import lombok.val;

public final class DeltaTimeCalculator {
  private double lastTimestamp = Timer.getTimestamp();

  public double get() {
    val currentTimestamp = Timer.getTimestamp();
    val delta = currentTimestamp - lastTimestamp;
    lastTimestamp = currentTimestamp;
    return delta;
  }

  public Time getMeasure() {
    return Seconds.of(get());
  }
}
