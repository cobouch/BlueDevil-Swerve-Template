package com.frc6324.template.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;

public final class DeltaTimeCalculator {
  private double lastTimestamp = RobotController.getFPGATime() / 1e6;

  public double get() {
    double newTimestamp = RobotController.getFPGATime() / 1e6;
    double delta = newTimestamp - lastTimestamp;
    lastTimestamp = newTimestamp;
    return delta;
  }

  public Time getMeasure() {
    return Seconds.of(get());
  }
}
