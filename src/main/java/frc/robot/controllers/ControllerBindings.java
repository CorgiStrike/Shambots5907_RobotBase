package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class to create controller bindings in Name them descriptively (default Trigger shoot(), etc.)
 * Generally, all of these methods should be defaulted. This makes it so you only need to override
 * bindings for modes where you want that control (there are often several controls that aren't
 * necessary for sim)
 */
public interface ControllerBindings {

  default double getDriveXValue() {
    return 0;
  }

  default double getDriveYValue() {
    return 0;
  }

  default double getDriveTurnValue() {
    return 0;
  }

  default Trigger tuningIncrement() {
    return new Trigger(() -> false);
  }

  default Trigger tuningDecrement() {
    return new Trigger(() -> false);
  }

  default Trigger tuningStop() {
    return new Trigger(() -> false);
  }

  default Trigger traverse() {
    return new Trigger(() -> false);
  }

  default Trigger xShape() {
    return new Trigger(() -> false);
  }

  default Trigger resetGyro() {
    return new Trigger(() -> false);
  }

  default void setRumble(double rumbleValue) {}
}
