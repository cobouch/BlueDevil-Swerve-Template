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
import lombok.Getter;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** A logged number on the dashboard, with a default value */
public class LoggedTunableNumber {
  @Nullable private final LoggedNetworkNumber dashboardNumber;

  /**
   * If {@link Constants#DEVBOT} is true, then this represents the value at the last time {@link
   * #update()} or {@link #get()}, or the default value if neither of these has been called.
   * Otherwise, if {@link Constants#DEVBOT} is false, this is equivalent to the default value.
   */
  @Getter private double cachedValue;

  /**
   * Creates a new logged number.
   *
   * @param key The key to put this number at when {@link Constants#DEVBOT} is true.
   * @param defaultValue The default value of this number.
   */
  public LoggedTunableNumber(String key, double defaultValue) {
    if (Constants.DEVBOT) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    } else {
      dashboardNumber = null;
    }
    cachedValue = defaultValue;
  }

  /**
   * Updates this number and returns what is on the dashboard, or the default if {@link
   * Constants#DEVBOT} is false.
   *
   * @return The most current value.
   */
  public double get() {
    update();
    return cachedValue;
  }

  /**
   * Checks if this dashboard number has changed, but does not update it.
   *
   * @return Whether this value has been changed on the dashboard.
   */
  public boolean hasChanged() {
    if (dashboardNumber != null) {
      return cachedValue == dashboardNumber.get();
    } else {
      return false;
    }
  }

  /**
   * Updates the cached value held by this object to the same that the dashboard has. This does
   * nothing if {@link Constants#DEVBOT} is false.
   */
  public void update() {
    if (dashboardNumber != null) {
      cachedValue = dashboardNumber.get();
    }
  }
}
