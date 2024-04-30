package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorInputs {
        public double elevatorPosition = 0.0;
        public double elevatorTarget = 0.0;
        public double encoderPosition = 0.0;

        public double elevatorRotorVelocity = 0.0;
        public double elevatorVelocity = 0.0;
        public double elevatorVoltage = 0.0;

        public double followerPosition = 0.0;
        public double followerVelocity = 0.0;
    }

    public default void updateInputs(ElevatorInputs inputs){}

    public default void setTargetPosition(double position){}

    public default void stop(){}

    public default void syncToAbsoluteEncoder(){}

    public default void resetFollower(){}

    public default void setGains(PIDSVGains gains){}

    public default void setVoltage(double voltage){}
}
