package com.frc6324.template.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.frc6324.template.subsystems.drive.SwerveDrive;
import com.frc6324.template.util.AllianceFlipUtil;
import com.frc6324.template.util.LoggedProfiledPID;
import com.frc6324.template.util.PoseSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.val;
import org.littletonrobotics.junction.Logger;

/**
 * A command that drives in a straight line to a target pose. This command intentionally ignores
 * obstacles; in the case where the driving scenario is complex (such as defense robots or many game
 * pieces in the way), this command should be disregarded in favor of the driver navigating
 * themselves.
 *
 * <p>The PID values for each instance for this command can be found on SmartDashboard if {@link
 * org.salemrobotics.frc.Constants#DEVBOT} is true. This command also logs a large amount of control
 * information for debugging purposes.
 *
 * @see Command
 * @see LoggedProfiledPID
 */
public class DriveToPoseCommand extends Command {
  private static final double DRIVE_KP = 2;
  private static final double DRIVE_KI = 0;
  private static final double DRIVE_KD = 1e-3;
  private static final double DRIVE_MAX_VELOCITY = 4.73;
  private static final double DRIVE_MAX_ACCELERATION = 5;
  private static final double TURN_KP = 2;
  private static final double TURN_KI = 0;
  private static final double TURN_KD = 5e-4;
  private static final double TURN_MAX_VELOCITY = 8;
  private static final double TURN_MAX_ACCELERATION = 20;
  private static final double TRANSLATION_TOLERANCE = Units.inchesToMeters(2);
  private static final double ROTATION_TOLERANCE = Units.degreesToRadians(1.5);

  private final String logKey;
  private final PoseSupplier poseSupplier;
  private final SwerveDrive drive;
  private final Supplier<Translation2d> driverOffset;

  private final LoggedProfiledPID translationController;
  private final LoggedProfiledPID rotationController;

  private Translation2d targetTranslation;
  private Rotation2d targetRotation;

  /**
   * Creates a command that drives to the pose returned by a given supplier.
   *
   * @param drive The drivetrain to command.
   * @param name The name of this command (e.g. "DriveToStation", "DriveToCage").
   * @param poseSupplier The supplier of the target to approach. This is called once during this
   *     command's initialization each time it is scheduled.
   * @param xOffsetSupplier The supplier of the driver's X joystick input bounded within [0, 1].
   * @param yOffsetSupplier The supplier of the driver's Y joystick input bounded within [0, 1].
   */
  public DriveToPoseCommand(
      SwerveDrive drive,
      String name,
      PoseSupplier poseSupplier,
      DoubleSupplier xOffsetSupplier,
      DoubleSupplier yOffsetSupplier) {
    setName(name);
    this.logKey = "AutoAlign/" + name;

    translationController =
        new LoggedProfiledPID(
            logKey + "/TranslationPID",
            DRIVE_KP,
            DRIVE_KI,
            DRIVE_KD,
            DRIVE_MAX_VELOCITY,
            DRIVE_MAX_ACCELERATION);
    translationController.setGoal(0);
    translationController.setTolerance(TRANSLATION_TOLERANCE);

    rotationController =
        new LoggedProfiledPID(
            logKey + "/RotationPID",
            TURN_KP,
            TURN_KI,
            TURN_KD,
            TURN_MAX_VELOCITY,
            TURN_MAX_ACCELERATION);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(ROTATION_TOLERANCE);

    this.drive = drive;
    this.poseSupplier = poseSupplier;
    this.driverOffset =
        () -> {
          var translation =
              DriveCommands.getLinearVelocityFromJoysticks(
                      xOffsetSupplier.getAsDouble(), yOffsetSupplier.getAsDouble())
                  .times(drive.getMaxLinearSpeed().in(MetersPerSecond) / 2.5);
          if (AllianceFlipUtil.shouldFlip()) {
            translation = translation.rotateBy(Rotation2d.k180deg);
          }

          return translation;
        };

    Logger.recordOutput(logKey + "/Running", false);
    addRequirements(drive);
  }

  /**
   * Constructs a command that drives to the pose returned by a given supplier.
   *
   * @param drive The drivetrain to command.
   * @param name The name of this command (e.g. "DriveToStation", "DriveToCage").
   * @param poseSupplier The supplier of the target to approach. This is called once during this
   *     command's initialization each time it is scheduled.
   */
  public DriveToPoseCommand(SwerveDrive drive, String name, PoseSupplier poseSupplier) {
    setName(name);
    this.logKey = "AutoAlign/" + name;

    translationController =
        new LoggedProfiledPID(
            logKey + "/TranslationConstants",
            DRIVE_KP,
            DRIVE_KI,
            DRIVE_KD,
            DRIVE_MAX_VELOCITY,
            DRIVE_MAX_ACCELERATION);
    translationController.setGoal(0);
    translationController.setTolerance(TRANSLATION_TOLERANCE);

    rotationController =
        new LoggedProfiledPID(
            logKey + "/RotationConstants",
            TURN_KP,
            TURN_KI,
            TURN_KD,
            TURN_MAX_VELOCITY,
            TURN_MAX_ACCELERATION);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(ROTATION_TOLERANCE);

    this.drive = drive;
    this.poseSupplier = poseSupplier;

    driverOffset = () -> Translation2d.kZero;

    Logger.recordOutput(logKey + "/Running", false);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Check for updates to PID
    translationController.updatePID();
    rotationController.updatePID();

    // Get the target pose's translation and rotation
    val targetPose = poseSupplier.getPose2d();
    targetTranslation = targetPose.getTranslation();
    targetRotation = targetPose.getRotation();

    // Reset the PID controllers
    translationController.setGoal(0);
    translationController.reset(distanceToTarget());
    rotationController.reset(0);

    Logger.recordOutput(logKey + "/TargetPose", targetPose);
    Logger.recordOutput(logKey + "/DistanceToTarget", distanceToTarget());
    Logger.recordOutput(logKey + "/Running", true);
  }

  @Override
  public void execute() {
    // Get the drivetrain's current pose.
    val drivePose = drive.getPose();

    // Calculate the translational error and log it
    val errorVector = targetTranslation.minus(drivePose.getTranslation());
    Logger.recordOutput(logKey + "/ErrorVector", errorVector);
    Logger.recordOutput(logKey + "/DistanceToTarget", errorVector.getNorm());

    // Use the translation PID controller to calculate a target linear magnitude and log it.
    // The PID controller does NOT account for direction and the magnitude is assumed to be in the X
    // direction
    // until it is rotated by the original error vector.
    val linearMagnitude = translationController.calculate(errorVector.getNorm());
    Logger.recordOutput(logKey + "/LinearOutput", linearMagnitude);

    // Construct the error with (pidOutput, errorAngle) and add the driver offset to it.
    val commandedTranslation =
        new Translation2d(linearMagnitude, errorVector.getAngle()).plus(driverOffset.get());

    // Calculate the heading error
    val angularError = drivePose.getRotation().minus(targetRotation);
    Logger.recordOutput(logKey + "/AngularError", angularError);

    // Use the angle PID controller to calculate omega
    val omega =
        rotationController.calculate(
            drivePose.getRotation().getRadians(), targetRotation.getRadians());
    Logger.recordOutput(logKey + "/AngularOutput", omega);

    // Send the command to the drivetrain
    drive.runFieldCentric(
        new ChassisSpeeds(commandedTranslation.getX(), commandedTranslation.getY(), omega));
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput(logKey + "/Running", false);
  }

  /**
   * Gets the distance from the robot's pose to the target pose.
   *
   * @return The distance, in meters, from the robot's translation to the target pose.
   */
  public double distanceToTarget() {
    return targetTranslation.minus(drive.getPose().getTranslation()).getNorm();
  }
}
