package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.SwerveDrive;

import java.util.function.Supplier;

import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends StateMachine<Drivetrain.State> {
    private SwerveDrive serveDrive;

    private Supplier<Double> xSupplier;
    private Supplier<Double> ySupplier;
    private Supplier<Double> thetaSupplier;

    public Drivetrain(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> thetaSupplier) {
        super("Drivetrain", State.Undetermined, State.class);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.thetaSupplier = thetaSupplier;

        this.serveDrive = new SwerveDrive(
                PIGEON_CAN_ID,
                Modules.DRIVE_GAINS,
                Modules.TURN_GAINS,
                MAX_CHASSIS_SPEED,
                MAX_CHASSIS_ACCELERATION,
                Modules.MAX_TURN_SPEED,
                Modules.MAX_TURN_ACCELERATION,
                TELEOP_THETA_GAINS,
                AUTO_THETA_GAINS,
                TRANSLATION_GAINS,
                false, //Set to true for extra telemetry
                Modules.CAN_BUS,
                GYRO_CAN_BUS,
                Constants.CURRENT_LIMITS_CONFIGS,
                Modules.MODULE_1,
                Modules.MODULE_2,
                Modules.MODULE_3,
                Modules.MODULE_4
        );
    }

    private void registerStates() {
        registerStateCommand(State.TeleopFieldOriented, new DriveCommand(
                serveDrive,
                xSupplier,
                ySupplier,
                thetaSupplier,
                Constants.Controller.DEADBAND,
                Constants.Controller.CONVERTER,

        ));
    }

    @Override
    protected void determineSelf() {
        setState(State.Idle);
    }

    public enum State {
        Undetermined,
        Idle,
        TeleopFieldOriented,
        TeleopBotOriented
    }
}
