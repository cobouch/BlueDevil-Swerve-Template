package com.frc6324.template.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;

import com.frc6324.template.generated.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public final class DriveIOSim extends DriveIOCTRE {
  private static final double SIM_LOOP_PERIOD = 0.004;

  @SuppressWarnings("unchecked")
  private final MapleSimDriveBase simulation =
      new MapleSimDriveBase(
          Seconds.of(SIM_LOOP_PERIOD),
          getPigeon2(),
          getModules(),
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  private final Notifier notifier = new Notifier(simulation::update);

  public DriveIOSim() {
    super();

    registerTelemetry(state -> state.Pose = simulation.getSimulatedDriveTrainPose());

    notifier.setName("Simulation Thread");
    notifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  @Override
  public void resetPose(Pose2d pose) {
    simulation.setSimulationWorldPose(pose);
    Timer.delay(0.05);

    super.resetPose(pose);
  }
}
