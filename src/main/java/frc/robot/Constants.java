package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.swerve.ModuleInfo;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.ModuleInfo.SwerveModuleSpeedLevel;
import frc.robot.ShamLib.swerve.ModuleInfo.SwerveModuleType;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public final class Constants {
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs();

    public static final class Controller {
        public static final double DEADBAND = 0.025;
        public static final UnaryOperator<Double> CONVERTER = (input) -> (Math.copySign(input * input, input));
    }

    public static final class Drivetrain {
        //TODO: SET GYRO CAN INFO
        public static final String GYRO_CAN_BUS = "";
        public static final int PIGEON_CAN_ID = 1;

        //TODO: SET TRACK WIDTH AND WHEEL_BASE
        // Distance between centers of right and left wheels on robot in meters
        public static final double TRACK_WIDTH = Units.inchesToMeters(0);
        // Distance between front and back wheels on robot in meters
        public static final double WHEEL_BASE = Units.inchesToMeters(0);

        public static final double rotationRadius = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;

        // Standard speeds (MK4 L3 modules capable of 5.27 m/s)
        public static final double LINEAR_SPEED = 5.27;
        public static final double LINEAR_ACCELERATION = 10;
        public static final double ROTARY_SPEED = (LINEAR_SPEED / rotationRadius) * (2 * Math.PI);
        public static final double ROTARY_ACCELERATION = ROTARY_SPEED * 3;

        public static final SwerveSpeedLimits MAX_SWERVE_LIMITS = new SwerveSpeedLimits(
                LINEAR_SPEED,
                LINEAR_ACCELERATION,
                ROTARY_SPEED,
                ROTARY_ACCELERATION
        );

        public static final SwerveModuleState[] X_SHAPE = new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

        //TODO: TUNE AUTO PID GAINS
        public static final PIDGains AUTO_THETA_GAINS = new PIDGains(0, 0, 0);
        public static final PIDGains AUTO_TRANSLATION_GAINS = new PIDGains(0, 0, 0);

        public static final class Modules {
            public static final String CAN_BUS = "";

            public static final double WHEEL_X_OFFSET = TRACK_WIDTH/2;
            public static final double WHEEL_Y_OFFSET = WHEEL_BASE/2;

            //These are values we used in 2023. They seem to work with no issues
            public static final double MAX_TURN_SPEED = 1000;
            public static final double MAX_TURN_ACCELERATION = 1000;

            //TODO: Specify information about what type of modules you're running
            public static final SwerveModuleType MODULE_TYPE = SwerveModuleType.MK4;
            public static final SwerveModuleSpeedLevel SPEED_LEVEL = SwerveModuleSpeedLevel.L3;

            //TODO: CALCULATE TURN AND DRIVE GAINS
            //TODO: DOCUMENTATION FOR FALCON AND SWERVE TUNING PROCESS
            public static final PIDSVGains DRIVE_GAINS = new PIDSVGains(
                0,
                0,
                0,
                0,
                0
            );

            public static final PIDSVGains TURN_GAINS = new PIDSVGains(
                0,
                0,
                0,
                0,
                0
            );

            //TODO: FILL IN MODULE CAN AND ENCODER OFFSET INFO
            //TODO: You may have to change whether the drive motors are inverted or not. The existing inversions should work for a standard MK4

            //front left
            public static final ModuleInfo MODULE_1 = ModuleInfo.generateModuleInfo(
                MODULE_TYPE, SPEED_LEVEL,
                    0,
                    0,
                    0,
                    0,
                    new Translation2d(WHEEL_X_OFFSET, WHEEL_Y_OFFSET),
                    false
            );

            //back left
            public static final ModuleInfo MODULE_2 = ModuleInfo.generateModuleInfo(
                MODULE_TYPE, SPEED_LEVEL,
                    0,
                    0,
                    0,
                    0,
                    new Translation2d(-WHEEL_X_OFFSET, WHEEL_Y_OFFSET),
                    false
            );

            //back right
            public static final ModuleInfo MODULE_3 = ModuleInfo.generateModuleInfo(
                MODULE_TYPE, SPEED_LEVEL,
                    0,
                    0,
                    0,
                    0,
                    new Translation2d(-WHEEL_X_OFFSET, -WHEEL_Y_OFFSET),
                    true
            );

            //front right
            public static final ModuleInfo MODULE_4 = ModuleInfo.generateModuleInfo(
                MODULE_TYPE, SPEED_LEVEL,
                    0,
                    0,
                    0,
                    0,
                    new Translation2d(WHEEL_X_OFFSET, -WHEEL_Y_OFFSET),
                    true
            );
        }
    }
}
