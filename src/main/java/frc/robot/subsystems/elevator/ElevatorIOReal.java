package frc.robot.subsystems.elevator;

import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import static frc.robot.Constants.Elevator.Hardware.*;
import static frc.robot.Constants.Elevator.Settings.*;


public class ElevatorIOReal implements ElevatorIO{
    
    protected final MotionMagicTalonFX leftMotor =
      new MotionMagicTalonFX(LEFT_ELEVATOR_ID, GAINS.get(), ELEVATOR_RATIO, ELEVATOR_VELOCITY, ELEVATOR_ACCELERATION, ELEVATOR_JERK);

    protected final MotionMagicTalonFX rightMotor =
      new MotionMagicTalonFX(RIGHT_ELEVATOR_ID, GAINS.get(), ELEVATOR_RATIO, ELEVATOR_VELOCITY, ELEVATOR_ACCELERATION, ELEVATOR_JERK);

}
