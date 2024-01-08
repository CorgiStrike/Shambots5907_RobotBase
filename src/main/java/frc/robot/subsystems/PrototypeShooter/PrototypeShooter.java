package frc.robot.subsystems.PrototypeShooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class PrototypeShooter extends StateMachine<PrototypeShooter.State> {
  private final PrototypeShooterIO io;
  private final PrototypeShooterInputsAutoLogged inputs = new PrototypeShooterInputsAutoLogged();

  private double velocityTarget = 0;
  private double voltageTarget = 0;

  public PrototypeShooter(
      PrototypeShooterIO io,
      Trigger tuningIncremnetTrigger,
      BooleanSupplier tuningInterruptSupplier) {
    super("Prototype Shooter", State.UNDETERMINED, State.class);

    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands(tuningIncremnetTrigger, tuningInterruptSupplier);
    registerTransitions();
  }

  private double getVelocityTarget() {
    return velocityTarget;
  }

  private double getVoltageTarget() {
    return voltageTarget;
  }

  private void setVelocityTarget(double target) {
    velocityTarget = target;
  }

  private void setVoltageTarget(double target) {
    voltageTarget = target;
  }

  private void registerStateCommands(
      Trigger tuningIncremnetTrigger, BooleanSupplier tuningInterruptSupplier) {
    registerStateCommand(
        State.VELOCITY_CONTROL,
        new FunctionalCommand(
            () -> {},
            () -> {
              double velocityTarget = getVelocityTarget();

              io.setMotorVelocityTarget(velocityTarget);

              if (inputs.motorVelocity == velocityTarget) {
                setFlag(State.AT_TARGET_VELOCITY);
              } else {
                clearFlag(State.AT_TARGET_VELOCITY);
              }
            },
            (interrupted) -> {
              io.setMotorVelocityTarget(0);
            },
            () -> false));

    registerStateCommand(
        State.VOLTAGE_CONTROL,
        new FunctionalCommand(
            () -> {},
            () -> {
              double velocityTarget = getVelocityTarget();

              io.setMotorVoltage(velocityTarget);
            },
            (interrupted) -> {
              io.setMotorVoltage(0);
            },
            () -> false));

    registerStateCommand(
        State.IDLE,
        new InstantCommand(
            () -> {
              io.setMotorVelocityTarget(0);
              io.setMotorVoltage(0);
              setVelocityTarget(0);
              setVoltageTarget(0);
            }));

    registerStateCommand(
        State.CALCULATE_KS,
        new SequentialCommandGroup(
            io.calculateKS(tuningIncremnetTrigger, tuningInterruptSupplier),
            transitionCommand(State.IDLE)));

    registerStateCommand(
        State.CALCULATE_KV,
        new SequentialCommandGroup(
            io.calculateKV(tuningIncremnetTrigger, tuningInterruptSupplier),
            transitionCommand(State.IDLE)));
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE);

    addCommutativeTransition(State.IDLE, State.VELOCITY_CONTROL, new InstantCommand());
    addCommutativeTransition(State.IDLE, State.CALCULATE_KS, new InstantCommand());
    addCommutativeTransition(State.IDLE, State.CALCULATE_KV, new InstantCommand());
    addCommutativeTransition(State.IDLE, State.VOLTAGE_CONTROL, new InstantCommand());
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  @Override
  protected void additionalSendableData(SendableBuilder builder) {
    builder.addDoubleProperty("Velocity Target", this::getVelocityTarget, this::setVelocityTarget);
    builder.addDoubleProperty("Voltage Target", this::getVoltageTarget, this::setVoltageTarget);
  }

  @Override
  public Map<String, Sendable> additionalSendables() {
    return Map.of(
        "Switch to Voltage Control", transitionCommand(State.VOLTAGE_CONTROL),
        "Switch to Velocity Control", transitionCommand(State.VELOCITY_CONTROL),
        "Switch to KS Calc", transitionCommand(State.CALCULATE_KS),
        "Switch to KV Calc", transitionCommand(State.CALCULATE_KV),
        "Switch to Idle", transitionCommand(State.IDLE));
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    VELOCITY_CONTROL,
    VOLTAGE_CONTROL,
    CALCULATE_KS,
    CALCULATE_KV,

    // Flags
    AT_TARGET_VELOCITY
  }
}
