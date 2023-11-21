package frc.robot.subsystems;

import frc.robot.ShamLib.SMF.StateMachine;

public class Drivetrain extends StateMachine<Drivetrain.State> {
    public Drivetrain() {
        super("Drivetrain", State.Undetermined, State.class);
    }

    @Override
    protected void determineSelf() {

    }

    public enum State {
        Undetermined
    }
}
