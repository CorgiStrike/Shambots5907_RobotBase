package frc.robot;

import static frc.robot.Constants.Lights.Hardware.NUM_LIGHTS;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.ShamLib.Candle.RGB;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.tuning.LoggedTunablePIDSV;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleSpeedLevel;
import frc.robot.ShamLib.swerve.module.ModuleInfo.SwerveModuleType;
import frc.robot.ShamLib.swerve.odometry.OdometryBoundingBox;
import frc.robot.subsystems.vision.Vision.CamSettings;
import java.util.function.UnaryOperator;
import frc.robot.ShamLib.motors.tuning.LoggedTunablePIDSV;

public final class Constants {

  public static final double LOOP_PERIOD = 0.02;
  private static final boolean ALLOW_TUNING = true;

  public static BuildMode currentBuildMode = BuildMode.REAL;

  public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs();

  // TODO: Update these for each year
  public static final double AUTO_TIME = 15;
  public static final double GAP_TIME = 3; // Time between autonomous and teleop start
  public static final double TELE_TIME = 135;
  public static final double ENDGAME_TIME = 20;

  public static final class Controller {
    public static final int LEFT_FLIGHT_STICK_ID = 1;
    public static final int RIGHT_FLIGHT_STICK_ID = 2;
    public static final int OPERATOR_CONTROLLER_ID = 0;

    public static final double DEADBAND = 0.075;
    public static final UnaryOperator<Double> DRIVE_CONVERSION =
        (input) -> (Math.copySign(input * input, input));

    // Different shuffleboard tabs to switch thorugh
    public static final String AUTO_SHUFFLEBOARD_TAB = "Auto";
    public static final String TELE_SHUFFLEBOARD_TAB_ID = "Tele";
    public static final String TEST_SHUFFLEBOARD_TAB_ID = "Test";
    public static final String TUNE_SHUFFLEBOARD_TAB_ID = "Tune";

    // Voltage below which the operator controller wills tart rumbling as a warning
    public static final double VOLTAGE_WARNING = 7.5;
  }

  public static final class PhysicalConstants {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    public static final OdometryBoundingBox FIELD_BOUNDING_BOX =
        new OdometryBoundingBox(
            new Translation2d(), new Translation2d(Units.feetToMeters(54), Units.feetToMeters(27)));

    static {
      try {
        // TODO: Update the Apriltag layout for the current season
        APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (Exception e) {
        throw new RuntimeException("Could not load AprilTag field layout from WPI");
      }
    }
  }

  public static final class Drivetrain {
    // TODO: SET GYRO CAN INFO
    public static final String GYRO_CAN_BUS = "";
    public static final int PIGEON_CAN_ID = 1;

    // TODO: SET TRACK WIDTH AND WHEEL_BASE
    // Distance between centers of right and left wheels on robot in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(0);
    // Distance between front and back wheels on robot in meters
    public static final double WHEEL_BASE = Units.inchesToMeters(0);

    public static final double rotationRadius =
        Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;

    // Standard speeds (MK4 L3 modules capable of 5.27 m/s)
    public static final double LINEAR_SPEED = 5.27;
    public static final double LINEAR_ACCELERATION = 10;
    public static final double ROTARY_SPEED = (LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
    public static final double ROTARY_ACCELERATION = ROTARY_SPEED * 3;

    public static final SwerveSpeedLimits MAX_SWERVE_LIMITS =
        new SwerveSpeedLimits(LINEAR_SPEED, LINEAR_ACCELERATION, ROTARY_SPEED, ROTARY_ACCELERATION);

    public static final Matrix<N3, N1> STATE_STD_DEVIATIONS = VecBuilder.fill(0.003, 0.003, 0.0002);

    public static final SwerveModuleState[] X_SHAPE =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

    // TODO: TUNE AUTO PID GAINS
    public static final PIDGains AUTO_THETA_GAINS = new PIDGains(0, 0, 0);
    public static final PIDGains AUTO_TRANSLATION_GAINS = new PIDGains(0, 0, 0);

    // radians
    public static final double FACE_ANGLE_TOLERANCE = 0.02;

    public static final double TURN_VOLTAGE_INCREMENT = 0.125;
    public static final double DRIVE_VOLTAGE_INCREMENT = 0.125;

    public static final class Modules {
      public static final String CAN_BUS = "";

      public static final double WHEEL_X_OFFSET = TRACK_WIDTH / 2;
      public static final double WHEEL_Y_OFFSET = WHEEL_BASE / 2;

      // These are values we used in 2023. They seem to work with no issues
      public static final double MAX_TURN_SPEED = 1000;
      public static final double MAX_TURN_ACCELERATION = 1000;

      // TODO: Specify information about what type of modules you're running
      public static final SwerveModuleType MODULE_TYPE = SwerveModuleType.MK4;
      public static final SwerveModuleSpeedLevel SPEED_LEVEL = SwerveModuleSpeedLevel.L3;

      // TODO: CALCULATE TURN AND DRIVE GAINS
      // TODO: DOCUMENTATION FOR FALCON AND SWERVE TUNING PROCESS
      public static final PIDSVGains DRIVE_GAINS = new PIDSVGains(0, 0, 0, 0, 0);

      public static final PIDSVGains TURN_GAINS = new PIDSVGains(0, 0, 0, 0, 0);

      // TODO: FILL IN MODULE CAN AND ENCODER OFFSET INFO
      // TODO: You may have to change whether the drive motors are inverted or not. The existing
      // inversions should work for a standard MK4

      // front left
      public static final ModuleInfo MODULE_1 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              0,
              0,
              0,
              0,
              new Translation2d(WHEEL_X_OFFSET, WHEEL_Y_OFFSET),
              false,
              true,
              true);

      // back left
      public static final ModuleInfo MODULE_2 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              0,
              0,
              0,
              0,
              new Translation2d(-WHEEL_X_OFFSET, WHEEL_Y_OFFSET),
              false,
              true,
              true);

      // back right
      public static final ModuleInfo MODULE_3 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              0,
              0,
              0,
              0,
              new Translation2d(-WHEEL_X_OFFSET, -WHEEL_Y_OFFSET),
              true,
              true,
              true);

      // front right
      public static final ModuleInfo MODULE_4 =
          ModuleInfo.generateModuleInfo(
              MODULE_TYPE,
              SPEED_LEVEL,
              0,
              0,
              0,
              0,
              new Translation2d(WHEEL_X_OFFSET, -WHEEL_Y_OFFSET),
              true,
              true,
              true);
    }
  }

  public static final class Vision {
    public static final class Sim {}

    public static final class Hardware {

      public static Pose3d EXAMPLE_CAM_POSE =
          new Pose3d(
              Units.inchesToMeters(0),
              Units.inchesToMeters(0),
              Units.inchesToMeters(0),
              new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
    }

    public static final class Settings {
      public static final int LIMELIGHT_NOTE_TRACK_PIPELINE = 0;

      public static final double AUTO_START_TOLERANCE = 0.5;

      public static final double EXAMPLE_CAM_TRUST_CUTOFF = Units.feetToMeters(18);

      public static final CamSettings EXAMPLE_CAM_SETTINGS =
          new CamSettings(Hardware.EXAMPLE_CAM_POSE, EXAMPLE_CAM_TRUST_CUTOFF, 0.4, 2.0, 0.33, 1.0);
    }
  }

  public static final class Elevator{
    public static final class Hardware{
      public static final int LEFT_ELEVATOR_ID = 2;
      public static final int RIGHT_ELEVATOR_ID = 1;

      public static final boolean LEFT_INVERTED = false;
      public static final boolean RIGHT_INVERTED = true;

      
    }

    public static final class Settings{
      
      public static final double ELEVATOR_VELOCITY = 4000;
      public static final double ELEVATOR_ACCELERATION = 1200;
      public static final double ELEVATOR_JERK = 500;

      public static final double ELEVATOR_RATIO = (1/6);

      public static final double ELEVATOR_FX_POSITION = 0;

      public static final double ELEVATOR_ROTOR_VELOCITY = 0;
      public static final double ELEVATOR_FOLLOWER_POSITION = 0;
      public static final double ELEVATOR_FOLLOWER_VELOCITY = 0;
      public static final double ELEVATOR_TARGET = 0;
      public static final double ELEVATOR_ENCODER_POSITION = 0;
      public static final double ELEVATOR_HEIGHT = 0;
      

      public static final LoggedTunablePIDSV GAINS =
          new LoggedTunablePIDSV(
              "Climber Loaded Gains", new PIDSVGains(.5, 0, 0, 0.0138, 0.1225), () -> ALLOW_TUNING);

    }
  }


  public static final class Lights {
    public static final class Hardware {
      public static final int CANDLE_ID = 0;

      public static final int NUM_LIGHTS = 70;

      public static final double BRIGHTNESS = 1.0;
    }

    public static final class Settings {
      public static final int NUM_LIGHTS_WITHOUT_CANDLE = NUM_LIGHTS - 8;

      public static final double BOUNCE_SPEED = 0.5;
      public static final double BLINK_SPEED = .075;

      public static final RGB ERROR_RGB = new RGB(255, 0, 0);
      public static final RGB AUTO_RGB = new RGB(0, 0, 255);
      public static final RGB AUTO_BACKGROUND_RGB = new RGB(0, 0, 0);

      public static final RGB OFF_RGB = new RGB(0, 0, 0);

      public static final Animation DISABLED_ANIMATION =
          new LarsonAnimation(
              0, 0, 255, 0, BOUNCE_SPEED, NUM_LIGHTS_WITHOUT_CANDLE, BounceMode.Front, 7, 8);

      public static final Animation AUTO_ANIMATION =
          new ColorFlowAnimation(
              0, 0, 255, 0, .0125, NUM_LIGHTS, ColorFlowAnimation.Direction.Forward);

      public static final Animation TOGGLE_LOB_ANIMATION =
          new StrobeAnimation(255, 16, 240, 0, BLINK_SPEED * 1.5, NUM_LIGHTS);

      public static final Animation AUTOMATIC_SCORE_ANIMATION =
          new TwinkleAnimation(
              0, 0, 255, 0, 0.5, NUM_LIGHTS, TwinkleAnimation.TwinklePercent.Percent76);

      public static final Animation GRAB_RANDOM_NOTE_ANIMATION =
          new RainbowAnimation(1, .9, NUM_LIGHTS);
    }
  }

  public static boolean doubleEqual(double a, double b, double accuracy) {
    return Math.abs(a - b) < accuracy;
  }

  public static boolean doubleEqual(double a, double b) {
    return doubleEqual(a, b, 0.00001);
  }

  public static Pose2d mirror(Pose2d pose) {
    return new Pose2d(
        new Translation2d(
            PhysicalConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() - pose.getX(), pose.getY()),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
  }

  public static Rotation2d rotationBetween(Pose2d pose1, Pose2d pose2) {
    return Rotation2d.fromRadians(
        Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX()));
  }

  public static Pose3d convertPose2dToPose3d(Pose2d pose) {
    return new Pose3d(
        pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
  }
}
