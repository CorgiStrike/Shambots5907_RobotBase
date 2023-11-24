package frc.robot;

import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.motors.pro.PIDSVGains;
import frc.robot.ShamLib.swerve.ModuleInfo;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;

import java.util.function.UnaryOperator;

public final class Constants {
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs();

    public static final class Controller {
        public static final double DEADBAND = 0.025;
        public static final UnaryOperator<Double> CONVERTER = (input) -> (Math.copySign(input * input, input));
    }

    public static final class Drivetrain {
        public static final String GYRO_CAN_BUS = "";
        public static final int PIGEON_CAN_ID = 0;

        public static final SwerveSpeedLimits MAX_SWERVE_LIMITS = new SwerveSpeedLimits(
                0.0,
                0.0,
                0.0,
                0.0
        );

        public static final SwerveModuleState[] X_SHAPE = new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

        public static final PIDGains TELEOP_THETA_GAINS = new PIDGains(0, 0, 0);
        public static final PIDGains AUTO_THETA_GAINS = new PIDGains(0, 0, 0);
        public static final PIDGains TRANSLATION_GAINS = new PIDGains(0, 0, 0);

        public static final class Modules {
            public static final String CAN_BUS = "";

            public static final double MAX_TURN_SPEED = 0.0;
            public static final double MAX_TURN_ACCELERATION = 0.0;

            public static final PIDSVGains DRIVE_GAINS = new PIDSVGains(0, 0, 0, 0, 0);
            public static final PIDSVGains TURN_GAINS = new PIDSVGains(0, 0, 0, 0, 0);

            public static final ModuleInfo MODULE_1 = new ModuleInfo(
                    0,
                    0,
                    0,
                    0.0,
                    new Translation2d(0.0, 0.0),
                    0.0,
                    0.0,
                    false,
                    false
            );

            public static final ModuleInfo MODULE_2 = new ModuleInfo(
                    0,
                    0,
                    0,
                    0.0,
                    new Translation2d(0.0, 0.0),
                    0.0,
                    0.0,
                    false,
                    false
            );

            public static final ModuleInfo MODULE_3 = new ModuleInfo(
                    0,
                    0,
                    0,
                    0.0,
                    new Translation2d(0.0, 0.0),
                    0.0,
                    0.0,
                    false,
                    false
            );

            public static final ModuleInfo MODULE_4 = new ModuleInfo(
                    0,
                    0,
                    0,
                    0.0,
                    new Translation2d(0.0, 0.0),
                    0.0,
                    0.0,
                    false,
                    false
            );
        }
    }
}
