package frc.robot.subsystems.PrototypeShooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

public class PrototypeShooterIOReal implements PrototypeShooterIO {
  protected final VelocityTalonFX flywheelMotor;
  protected final PIDSVGains flywheelMotorGains;
  protected final double maxVelocity;

  public PrototypeShooterIOReal(
      int motorCanID,
      boolean motorInverted,
      double maxVelocity,
      PIDSVGains flywheelMotorGains,
      CurrentLimitsConfigs currentLimit) {
    this.flywheelMotorGains = flywheelMotorGains;
    this.maxVelocity = maxVelocity;

    flywheelMotor = new VelocityTalonFX(motorCanID, flywheelMotorGains, 1);
    flywheelMotor.setInverted(motorInverted);
    applyCurrentLimit(flywheelMotor, currentLimit);
  }

  @Override
  public void updateInputs(PrototypeShooterInputs inputs) {
    inputs.motorVelocity = this.flywheelMotor.getEncoderVelocity();
  }

  @Override
  public void applyCurrentLimit(TalonFX motor, CurrentLimitsConfigs limit) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    config.CurrentLimits = limit;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setMotorVelocityTarget(double target) {
    double normalizedTarget = Math.copySign(Math.min(Math.abs(target), maxVelocity), target);

    flywheelMotor.setTarget(normalizedTarget);
  }

  @Override
  public Command calculateKV(Trigger increment, BooleanSupplier interrupt) {
    return flywheelMotor.calculateKV(flywheelMotorGains.getS(), 0.5, increment, interrupt);
  }

  @Override
  public void setMotorVoltage(double voltage) {
    flywheelMotor.setVoltage(voltage);
  }

  @Override
  public Command calculateKS(Trigger increment, BooleanSupplier interrupt) {
    AtomicInteger currentMultiple = new AtomicInteger(0);
    increment.onTrue(new InstantCommand(currentMultiple::incrementAndGet));

    return new FunctionalCommand(
        () -> {
          System.out.println("Starting KS Calculation");
          currentMultiple.set(0);
        },
        () -> {
          double voltage = currentMultiple.get() * 0.125;

          flywheelMotor.setVoltage(voltage);

          System.out.println(
              "(KS) volts: " + voltage + ", velocity: " + flywheelMotor.getEncoderVelocity());
        },
        (interrupted) -> {
          flywheelMotor.stopMotor();
        },
        interrupt);
  }
}
