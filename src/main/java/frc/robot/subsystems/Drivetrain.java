package frc.robot.subsystems;

import static frc.robot.Constants.Drivetrain.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain.Modules;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.module.RealignModuleCommand;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends StateMachine<Drivetrain.State> {
  private final SwerveDrive swerveDrive;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaSupplier;

  @AutoLogOutput private SwerveModuleState[] moduleStates;

  public Drivetrain(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
    super("Drivetrain", State.Undetermined, State.class);

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.thetaSupplier = thetaSupplier;

    this.swerveDrive =
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
            Modules.MODULE_1,
            Modules.MODULE_2,
            Modules.MODULE_3,
            Modules.MODULE_4);

    registerStates();
    registerTransitions();
  }

  private void registerStates() {
    registerStateCommand(
        State.TeleopFieldOriented,
        new DriveCommand(
            swerveDrive,
            xSupplier,
            ySupplier,
            thetaSupplier,
            Constants.Controller.DEADBAND,
            Constants.Controller.CONVERTER,
            true,
            this,
            MAX_SWERVE_LIMITS));

    registerStateCommand(
        State.TeleopBotOriented,
        new DriveCommand(
            swerveDrive,
            xSupplier,
            ySupplier,
            thetaSupplier,
            Constants.Controller.DEADBAND,
            Constants.Controller.CONVERTER,
            true,
            this,
            MAX_SWERVE_LIMITS));

    registerStateCommand(
        State.XShape, new InstantCommand(() -> swerveDrive.setModuleStates(X_SHAPE)));

    registerStateCommand(State.Idle, new InstantCommand(swerveDrive::stopModules));
  }

  private void registerTransitions() {
    addOmniTransition(State.TeleopFieldOriented, () -> swerveDrive.setFieldRelative(true));
    addOmniTransition(State.TeleopBotOriented, () -> swerveDrive.setFieldRelative(false));
    addOmniTransition(State.XShape);
    addOmniTransition(State.Idle);
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public Command resetGyroCommand() {
    return new InstantCommand(this::resetGyro);
  }

  public void registerMisalignedSwerveTriggers(EventLoop loop) {
    for (SwerveModule module : swerveDrive.getModules()) {
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
        setState(State.TeleopFieldOriented);
      } else {
        setState(State.Idle);
      }
    } else {
      setState(State.Idle);
    }
  }

  @Override
  protected void update() {
    swerveDrive.update();

    moduleStates = swerveDrive.getModuleStates();

    Logger.recordOutput(getName() + "/pos ", getPose());

    // TODO: Add the limelight pose integration here using swerveDrive.addVisionMeasurement
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public enum State {
    Undetermined,
    Idle,
    TeleopFieldOriented,
    TeleopBotOriented,
    XShape
  }
}
