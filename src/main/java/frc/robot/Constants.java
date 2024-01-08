package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleSpeedLevel;
import frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleType;
import java.util.function.UnaryOperator;

public final class Constants {
  public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs();
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.REAL;

  public static final class Controller {
    public static final double DEADBAND = 0.025;
    public static final UnaryOperator<Double> CONVERTER =
        (input) -> (Math.copySign(input * input, input));
  }

  public static final class PROTOTYPE_SHOOTER {
    public static final int FLYWHEEL_CAN_ID = 11;
    public static final boolean INVERT_FLYWHEEL = false;
    public static final double MAX_VELOCITY = 5000; // RPM

    public static final PIDSVGains VELOCITY_GAINS = new PIDSVGains(0, 0, 0, 0, 0);
  }

  public static final class Drivetrain {
    public static final String GYRO_CAN_BUS = "";
    public static final int PIGEON_CAN_ID = 1;

    // Distance between centers of right and left wheels on robot in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot in meters
    public static final double WHEEL_BASE = Units.inchesToMeters(24);

    public static final double rotationRadius =
        Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;

    // Standard speeds (MK4 L3 modules capable of 5.27 m/s)
    public static final double LINEAR_SPEED = 5.27;
    public static final double LINEAR_ACCELERATION = 10;
    public static final double ROTARY_SPEED = (LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
    public static final double ROTARY_ACCELERATION = ROTARY_SPEED * 3;

    public static final SwerveSpeedLimits MAX_SWERVE_LIMITS =
        new SwerveSpeedLimits(LINEAR_SPEED, LINEAR_ACCELERATION, ROTARY_SPEED, ROTARY_ACCELERATION);

    public static final SwerveModuleState[] X_SHAPE =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

    public static final PIDGains AUTO_THETA_GAINS = new PIDGains(0, 0, 0);
    public static final PIDGains AUTO_TRANSLATION_GAINS = new PIDGains(0, 0, 0);

    public static final class Modules {
      public static final String CAN_BUS = "";

      public static final double WHEEL_X_OFFSET = TRACK_WIDTH / 2;
      public static final double WHEEL_Y_OFFSET = WHEEL_BASE / 2;

      public static final double MAX_TURN_SPEED = 1000;
      public static final double MAX_TURN_ACCELERATION = 1000;

      public static final SwerveModuleType MODULE_TYPE = SwerveModuleType.MK4;
      public static final SwerveModuleSpeedLevel SPEED_LEVEL = SwerveModuleSpeedLevel.L3;

      public static final PIDSVGains DRIVE_GAINS = new PIDSVGains(0.25, 0, 0, 0.3, 0.1135);

      public static final PIDSVGains TURN_GAINS = new PIDSVGains(10, 0, 0, 0.3, 0.121057);

      // front left
      public static final ModuleInfo MODULE_1 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              1,
              2,
              1,
              -25.752,
              new Translation2d(WHEEL_X_OFFSET, WHEEL_Y_OFFSET),
              false);

      // back left
      public static final ModuleInfo MODULE_2 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              3,
              4,
              3,
              -165.1,
              new Translation2d(-WHEEL_X_OFFSET, WHEEL_Y_OFFSET),
              false);

      // back right
      public static final ModuleInfo MODULE_3 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              5,
              6,
              5,
              75.6,
              new Translation2d(-WHEEL_X_OFFSET, -WHEEL_Y_OFFSET),
              true);

      // front right
      public static final ModuleInfo MODULE_4 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              7,
              8,
              7,
              -84.9,
              new Translation2d(WHEEL_X_OFFSET, -WHEEL_Y_OFFSET),
              true);
    }
  }
}
