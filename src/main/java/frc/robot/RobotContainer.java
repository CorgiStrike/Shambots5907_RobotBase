// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final CommandFlightStick leftStick = new CommandFlightStick(0);
  private final CommandFlightStick rightStick = new CommandFlightStick(1);
  private final CommandXboxController operatorCont = new CommandXboxController(2);

  private final Drivetrain drivetrain;

  public RobotContainer(EventLoop checkModulesLoop) {
    super("Robot Container", State.Undetermined, State.class);

    drivetrain = new Drivetrain(
            () -> -leftStick.getY(),
            () -> -leftStick.getX(),
            () -> -rightStick.getX()
    );

    addChildSubsystem(drivetrain);

    //Have the drivetrain start checking for misaligned swerve modules
    drivetrain.registerMisalignedSwerveTriggers(checkModulesLoop);

    registerTransitions();

    configureBindings();
  }

  private void registerTransitions() {
    System.out.println("registering transitions");

    addOmniTransition(State.Disabled,
            drivetrain.transitionCommand(Drivetrain.State.Idle)
    );

    addOmniTransition(State.Teleoperated,
            drivetrain.transitionCommand(Drivetrain.State.TeleopFieldOriented)
    );

    addOmniTransition(State.Autonomous,
            drivetrain.transitionCommand(Drivetrain.State.Idle)
    );
  }

  private void configureBindings() {
    operatorCont.a().onTrue(drivetrain.transitionCommand(Drivetrain.State.TeleopFieldOriented));
    operatorCont.b().onTrue(drivetrain.transitionCommand(Drivetrain.State.TeleopBotOriented));
    operatorCont.x().onTrue(drivetrain.transitionCommand(Drivetrain.State.XShape));
    operatorCont.y().onTrue(drivetrain.transitionCommand(Drivetrain.State.Idle));

    leftStick.topBase().onTrue(drivetrain.resetGyroCommand());
  }

  @Override
  protected void onAutonomousStart() {
    requestTransition(State.Autonomous);
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.Teleoperated);
  }

  @Override
  protected void determineSelf() {
    if (DriverStation.isEnabled()) {
      if (DriverStation.isTeleop()) {
        setState(State.Teleoperated);
      }
      else {
        setState(State.Autonomous);
      }
    }
    else {
      setState(State.Disabled);
    }
  }

  public enum State {
    Undetermined,
    Disabled,
    Autonomous,
    Teleoperated,
  }
}
