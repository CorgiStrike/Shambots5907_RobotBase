// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.controllers.ControllerBindings;
import frc.robot.controllers.RealControllerBindings;
import frc.robot.controllers.SimControllerBindings;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsIO;
import frc.robot.subsystems.lights.LightsIOReal;
import frc.robot.subsystems.vision.Vision;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends StateMachine<RobotContainer.State> {

  // Declare subsystems
  private final Drivetrain drivetrain;
  private final Lights lights;
  private final Vision vision;

  // Declare power distribution for voltage and current monitoring/logging
  private PowerDistribution pd;

  // Autonomous Command Chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  // Controller bindings object that will be created to handle both real control and sim inputs
  private final ControllerBindings controllerBindings;

  // Store the previous drivetrain state so that x-shape command works properly
  private Drivetrain.State prevDTState = Drivetrain.State.FIELD_ORIENTED_DRIVE;

  // Track whether the bot has been enabled for autonomous check lights
  // Generally, you would only want to show auto condition status lights BEFORE the bot has been
  // enabled
  private boolean hasBeenEnabled = false;

  // Flag that indicates the vision pose is working
  // Should be toggle-able on the driver station
  private boolean poseWorking = true;

  public RobotContainer(EventLoop checkModulesLoop, PowerDistribution pd) {
    super("Robot Container", State.UNDETERMINED, State.class);

    this.pd = pd;

    if (Constants.currentBuildMode == ShamLibConstants.BuildMode.SIM) {
      controllerBindings = new SimControllerBindings();
    } else {
      controllerBindings = new RealControllerBindings();
    }

    Map<String, Vision.CamSettings> photonMap =
        Constants.currentBuildMode == BuildMode.SIM
            ? Map.of()
            : Map.of("example", Constants.Vision.Settings.EXAMPLE_CAM_SETTINGS);

    vision = new Vision("limelight", photonMap);

    drivetrain =
        new Drivetrain(
            controllerBindings::getDriveXValue,
            controllerBindings::getDriveYValue,
            controllerBindings::getDriveTurnValue,
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    vision.addVisionUpdateConsumers(drivetrain::addVisionMeasurements);

    vision.setOverallEstimateSupplier(drivetrain::getBotPose);

    lights = new Lights(getLightsIO(), () -> !hasBeenEnabled, autoConditions());

    addChildSubsystem(drivetrain);

    // Have the drivetrain start checking for misaligned swerve modules
    drivetrain.registerMisalignedSwerveTriggers(checkModulesLoop);

    registerStateCommands();
    registerTransitions();

    configureControllerBindings();

    registerNamedCommands();

    // Pathplanner must be configured AFTER all the named commands are set, otherwise the code will
    // crash
    drivetrain.configurePathplanner();

    // Important to instatiate after drivetrain consructor is called so that auto builder is
    // configured
    // TODO: Switch away from useing Pathplanner's built in auto stuff
    autoChooser =
        new LoggedDashboardChooser<>("Logged Autonomous Chooser", AutoBuilder.buildAutoChooser());
  }

  private void registerNamedCommands() {
    // TODO: Register all of your pathplanner Named Commands here
  }

  private void registerStateCommands() {
    System.out.println("---REGISTERING STATE COMMANDS---");

    registerStateCommand(
        State.TRAVERSING,
        new ParallelCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE)));
  }

  private void registerTransitions() {
    System.out.println("---REGISTERING TRANSITIONS---");

    addOmniTransition(State.TRAVERSING);

    addTransition(State.SOFT_E_STOP, State.AUTONOMOUS);
  }

  private void configureControllerBindings() {
    controllerBindings
        .traverse()
        .onTrue(drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE));

    controllerBindings
        .xShape()
        .onTrue(
            new InstantCommand(
                    () -> {
                      prevDTState = drivetrain.getState();
                    })
                .andThen(drivetrain.transitionCommand(Drivetrain.State.X_SHAPE)))
        .onFalse(new InstantCommand(() -> drivetrain.requestTransition(prevDTState)));

    controllerBindings.resetGyro().onTrue(drivetrain.resetGyroCommand());
  }

  private Trigger tuningIncrement() {
    return controllerBindings.tuningIncrement();
  }

  private Trigger tuningDecrement() {
    return controllerBindings.tuningDecrement();
  }

  private Trigger tuningStop() {
    return controllerBindings.tuningStop();
  }

  public Pose3d getBotPose() {
    Pose2d pose = drivetrain.getBotPose();
    return new Pose3d(
        new Translation3d(pose.getX(), pose.getY(), 0),
        new Rotation3d(0, 0, pose.getRotation().getRadians()));
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  @Override
  protected void onTestStart() {
    requestTransition(State.TEST);
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TRAVERSING);
  }

  @Override
  protected void onAutonomousStart() {

    Command selectedAutoCommand = autoChooser.get();

    String selectedAutoKey = autoChooser.getSendableChooser().getSelected();

    AtomicBoolean runningDelayPathfindAuto = new AtomicBoolean(false);

    if (selectedAutoKey.equals("4 Note")) runningDelayPathfindAuto.set(true);
    if (selectedAutoKey.equals("5 Note Adaptive")) runningDelayPathfindAuto.set(true);

    registerStateCommand(
        State.AUTONOMOUS,
        new SequentialCommandGroup(
            lights.transitionCommand(Lights.State.AUTO),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY),
            new InstantCommand(
                () -> {
                  selectedAutoCommand.schedule();
                })));

    requestTransition(State.AUTONOMOUS);
  }

  @Override
  protected void onEnable() {
    hasBeenEnabled = true;
  }

  @Override
  protected void onDisable() {
    controllerBindings.setRumble(0);
  }

  private LightsIO getLightsIO() {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new LightsIOReal();
      }

      default -> {
        return new LightsIO() {};
      }
    }
  }

  /** The conditions required for autonomous to be ready to start */
  public BooleanSupplier[] autoConditions() {
    // Cameras are initialized in the order 1,4,3,2 for some reason
    // This should be fixed in future years but I didn't want to modify it
    return new BooleanSupplier[] {
      () -> !hasBeenEnabled, () -> vision.isConnected(1), () -> DriverStation.isDSAttached()
    };
  }

  public enum State {
    UNDETERMINED,
    AUTONOMOUS,
    TRAVERSING,
    SOFT_E_STOP,
    TEST
  }
}
