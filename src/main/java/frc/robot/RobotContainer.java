// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ShamLib.SMF.StateMachine;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  public RobotContainer() {
    super("Robot Container", State.Undetermined, State.class);

    configureBindings();
  }

  private void configureBindings() {}

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
