package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.Drivetrain.*;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain.Modules;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.swerve.module.RealignModuleCommand;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends StateMachine<Drivetrain.State> {
  private final SwerveDrive drive;
  @AutoLogOutput private boolean flipPath = false;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaSupplier;

  @AutoLogOutput private SwerveModuleState[] moduleStates;

  public Drivetrain(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier theta,
      Trigger incrementUp,
      Trigger incrementDown,
      Trigger stop) {
    super("Drivetrain", State.UNDETERMINED, State.class);

    this.xSupplier = x;
    this.ySupplier = y;
    this.thetaSupplier = theta;

    this.drive =
        new SwerveDrive(
            Constants.currentBuildMode,
            PIGEON_CAN_ID,
            Modules.DRIVE_GAINS,
            Modules.TURN_GAINS,
            MAX_SWERVE_LIMITS.getMaxSpeed(),
            MAX_SWERVE_LIMITS.getMaxAcceleration(),
            MAX_SWERVE_LIMITS.getMaxRotationalSpeed(),
            MAX_SWERVE_LIMITS.getMaxRotationalAcceleration(),
            Modules.MAX_TURN_SPEED,
            Modules.MAX_TURN_ACCELERATION,
            AUTO_THETA_GAINS,
            AUTO_TRANSLATION_GAINS,
            false, // Set to true for extra telemetry
            Modules.CAN_BUS,
            GYRO_CAN_BUS,
            Constants.CURRENT_LIMITS_CONFIGS,
            this,
            false,
            () -> flipPath,
            Constants.Drivetrain.STATE_STD_DEVIATIONS,
            Constants.LOOP_PERIOD,
            Modules.MODULE_1,
            Modules.MODULE_2,
            Modules.MODULE_3,
            Modules.MODULE_4);

    registerStateCommands(stop, incrementUp, incrementDown);
    registerTransitions();

    // Make sure the alliaince flipping gets passed on to the drivetrain
    AllianceManager.addAllianceChangeHook(this::syncAlliance);

    PPHolonomicDriveController.setRotationTargetOverride(null);
  }

  public void syncAlliance() {
    // flip if we are on red alliance
    flipPath = AllianceManager.getAlliance() == DriverStation.Alliance.Red;

    // re-register face commands in case the alliance changed (they are based on the blue poses by
    // default)
    registerFaceCommands();

    registerAutoPathfindCommand();

    registerPathFollowStateCommands();

    // Helpful output to give drivers assurance that things are working properly
    System.out.println(AllianceManager.getAlliance());
    System.out.println("Path Flipping:" + flipPath);
  }

  public void addVisionMeasurements(
      List<TimestampedPoseEstimator.TimestampedVisionUpdate> measurement) {
    drive.addTimestampedVisionMeasurements(measurement);
  }

  private void registerStateCommands(Trigger stop, Trigger incrementUp, Trigger incrementDown) {
    registerStateCommand(
        State.FIELD_ORIENTED_DRIVE,
        new DriveCommand(
                drive,
                xSupplier,
                ySupplier,
                thetaSupplier,
                Constants.Controller.DEADBAND,
                Constants.Controller.DRIVE_CONVERSION,
                this,
                MAX_SWERVE_LIMITS)
            .alongWith(new InstantCommand(() -> setFlag(State.AT_ANGLE))));

    registerStateCommand(State.X_SHAPE, new InstantCommand(() -> drive.setModuleStates(X_SHAPE)));

    registerStateCommand(State.IDLE, new InstantCommand(drive::stopModules));

    registerStateCommand(
        State.TURN_VOLTAGE_CALC,
        drive.getTurnVoltageCalcCommand(stop, incrementUp, incrementDown, TURN_VOLTAGE_INCREMENT));
    registerStateCommand(
        State.DRIVE_VOLTAGE_CALC,
        drive.getDriveVoltageCalcCommand(
            stop, incrementUp, incrementDown, DRIVE_VOLTAGE_INCREMENT));
  }

  private void registerTransitions() {
    addOmniTransition(State.FIELD_ORIENTED_DRIVE, () -> drive.setFieldRelative(true));
    addOmniTransition(State.X_SHAPE);
    addOmniTransition(State.IDLE);

    addTransition(State.IDLE, State.FOLLOWING_AUTONOMOUS_TRAJECTORY);

    addTransition(State.IDLE, State.TURN_VOLTAGE_CALC);
    addTransition(State.IDLE, State.DRIVE_VOLTAGE_CALC);

    addTransition(State.IDLE, State.CALCULATE_WHEEL_RADIUS);
  }

  public void configurePathplanner() {
    drive.configurePathplanner();

    registerPathFollowStateCommands();

    registerAutoPathfindCommand();
  }

  // Factory syntax sugar because most of the time this isn't used in auto
  private Command getFacePointCommand(Pose2d pose, SwerveSpeedLimits limits) {
    return getFacePointCommand(pose, limits, false);
  }

  /**
   * Factory that generates a command to face a point on the field
   *
   * @param pose Location to face
   * @param limits Speed limits of the translation driving
   * @param usedInAuto Whether this is being run in autonomous or not
   * @return the command to run
   */
  private Command getFacePointCommand(Pose2d pose, SwerveSpeedLimits limits, boolean usedInAuto) {
    FacePointCommand facePointCommand =
        new FacePointCommand(
            drive,
            AUTO_THETA_GAINS,
            pose,
            drive::getPose,
            !usedInAuto ? xSupplier : () -> 0,
            !usedInAuto ? ySupplier : () -> 0,
            Constants.Controller.DEADBAND,
            Constants.Controller.DRIVE_CONVERSION,
            this,
            limits);

    return new ParallelCommandGroup(
        facePointCommand,
        new RunCommand(
            () -> {
              if (facePointCommand.atAngle(FACE_ANGLE_TOLERANCE)) {
                setFlag(State.AT_ANGLE);
              } else {
                clearFlag(State.AT_ANGLE);
              }
            }));
  }

  /** Register any command that involve pathfollowing here */
  private void registerPathFollowStateCommands() {}

  /** Register any commands that involve pathfinding in autonomous here */
  private void registerAutoPathfindCommand() {}

  /** Register any state commands that involve facing a specific point on the field */
  private void registerFaceCommands() {}

  public void resetGyro() {
    drive.resetGyro();
  }

  public Command resetGyroCommand() {
    return new InstantCommand(this::resetGyro);
  }

  public void registerMisalignedSwerveTriggers(EventLoop loop) {
    for (SwerveModule module : drive.getModules()) {
      loop.bind(
          () -> {
            if (module.isModuleMisaligned() && !isEnabled()) {
              new RealignModuleCommand(module).schedule();
            }
          });
    }
  }

  @Override
  protected void determineSelf() {
    if (DriverStation.isEnabled()) {
      if (DriverStation.isTeleop()) {
        setState(State.FIELD_ORIENTED_DRIVE);
      } else {
        setState(State.IDLE);
      }
    } else {
      setState(State.IDLE);
    }
  }

  @Override
  protected void update() {
    drive.update();

    moduleStates = drive.getModuleStates();

    Logger.recordOutput(getName() + "/pos ", getBotPose());

    // TODO: Add the limelight pose integration here using swerveDrive.addVisionMeasurement
  }

  public Pose2d getBotPose() {
    return drive.getPose();
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    FIELD_ORIENTED_DRIVE,
    X_SHAPE,
    FOLLOWING_AUTONOMOUS_TRAJECTORY,

    // Flags
    AT_ANGLE,

    // Tuning stuff
    TURN_VOLTAGE_CALC,
    DRIVE_VOLTAGE_CALC,
    CALCULATE_WHEEL_RADIUS,
  }
}
