// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShamLib.CommandFlightStick;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final CommandFlightStick leftStick = new CommandFlightStick(0);
  private final CommandFlightStick rightStick = new CommandFlightStick(1);
  private final CommandXboxController operatorCont = new CommandXboxController(2);

  private final Drivetrain drivetrain;

  public RobotContainer() {
    super("Robot Container", State.Undetermined, State.class);

    drivetrain = new Drivetrain(
            () -> -leftStick.getY(),
            () -> -leftStick.getX(),
            () -> -rightStick.getY()
    );

    addChildSubsystem(drivetrain);

    registerTransitions();

    configureBindings();
  }

  private void registerTransitions() {
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

    rightStick.trigger().onTrue(drivetrain.resetGyroCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
