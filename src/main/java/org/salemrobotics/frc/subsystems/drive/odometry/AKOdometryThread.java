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
package org.salemrobotics.frc.subsystems.drive.odometry;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class AKOdometryThread {
  public static final Lock GLOBAL_LOCK = new ReentrantLock();

  public static void addTask(Runnable task, BaseStatusSignal... signals) {}

  public static void start() {}

  public static double[] getTimestamps() {
    return new double[0];
  }
}
