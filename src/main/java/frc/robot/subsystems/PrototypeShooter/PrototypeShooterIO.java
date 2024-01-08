package frc.robot.subsystems.PrototypeShooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface PrototypeShooterIO {
  @AutoLog
  public static class PrototypeShooterInputs {
    public double motorVelocity = 0.0; // Rotations/s
    public double motorVelocityTarget = 0.0; // Rotations/s
  }

  public void updateInputs(PrototypeShooterInputs inputs);

  public default void applyCurrentLimit(TalonFX motor, CurrentLimitsConfigs limit) {}

  public void setMotorVelocityTarget(double target);

  public void setMotorVoltage(double voltage);

  public default Command calculateKV(Trigger increment, BooleanSupplier interrupt) {
    return new InstantCommand();
  }

  public default Command calculateKS(Trigger increment, BooleanSupplier interrupt) {
    return new InstantCommand();
  }
}
