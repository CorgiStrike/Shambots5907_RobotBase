package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SimControllerBindings implements ControllerBindings {
  private final CommandXboxController operatorController = new CommandXboxController(0);

  @Override
  public double getDriveXValue() {
    return -operatorController.getLeftY();
  }

  @Override
  public double getDriveYValue() {
    return -operatorController.getLeftX();
  }

  @Override
  public double getDriveTurnValue() {
    return -operatorController.getRightX();
  }
}
