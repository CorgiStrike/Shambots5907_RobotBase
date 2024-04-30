package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.ShamLib.SMF.StateMachine;

public class Arm extends StateMachine<Arm.State> {

  public Arm() {
    super("Arm", State.UNDETERMINED, State.class);

    registerStateCommand(State.IDLE, new PrintCommand("eat my shorts"));

    addTransition(State.IDLE, State.EXTENDED);
    addTransition(State.IDLE, State.HALF_EXTEND);

    addCommutativeTransition(getState(), getState(), new InstantCommand());

    addOmniTransition(State.IDLE);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public enum State {
    IDLE,
    EXTENDED,
    HALF_EXTEND,
    UNDETERMINED
  }
}
