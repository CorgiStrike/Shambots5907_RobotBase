package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.RealignModuleCommand;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveModule;

import java.util.Map;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends StateMachine<Drivetrain.State> {
    private final SwerveDrive swerveDrive;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier thetaSupplier;

    public Drivetrain(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        super("Drivetrain", State.Undetermined, State.class);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.thetaSupplier = thetaSupplier;

        this.swerveDrive = new SwerveDrive(
                PIGEON_CAN_ID,
                Modules.DRIVE_GAINS,
                Modules.TURN_GAINS,
                MAX_SWERVE_LIMITS.getMaxSpeed(),
                MAX_SWERVE_LIMITS.getMaxAcceleration(),
                Modules.MAX_TURN_SPEED,
                Modules.MAX_TURN_ACCELERATION,
                AUTO_THETA_GAINS,
                AUTO_TRANSLATION_GAINS,
                false, //Set to true for extra telemetry
                Modules.CAN_BUS,
                GYRO_CAN_BUS,
                Constants.CURRENT_LIMITS_CONFIGS,
                Modules.MODULE_1,
                Modules.MODULE_2,
                Modules.MODULE_3,
                Modules.MODULE_4
        );

        registerStates();
        registerTransitions();
    }

    private void registerStates() {
        registerStateCommand(State.TeleopFieldOriented, new DriveCommand(
                swerveDrive,
                xSupplier,
                ySupplier,
                thetaSupplier,
                Constants.Controller.DEADBAND,
                Constants.Controller.CONVERTER,
                true,
                this,
                MAX_SWERVE_LIMITS
        ));

        registerStateCommand(State.TeleopBotOriented, new DriveCommand(
                swerveDrive,
                xSupplier,
                ySupplier,
                thetaSupplier,
                Constants.Controller.DEADBAND,
                Constants.Controller.CONVERTER,
                true,
                this,
                MAX_SWERVE_LIMITS
        ));


        registerStateCommand(State.XShape, new InstantCommand(() ->
            swerveDrive.setModuleStates(X_SHAPE)
        ));

        registerStateCommand(State.Idle, new InstantCommand(
            swerveDrive::stopModules
        ));
    }

    private void registerTransitions() {
        addOmniTransition(State.TeleopFieldOriented, () -> swerveDrive.setFieldRelative(true));
        addOmniTransition(State.TeleopBotOriented, () -> swerveDrive.setFieldRelative(false));
        addOmniTransition(State.XShape);
        addOmniTransition(State.Idle);
    }

    public void resetGyro() {
        swerveDrive.resetGyro();
    }

    public Command resetGyroCommand() {
        return new InstantCommand(this::resetGyro);
    }

    public void registerMisalignedSwerveTriggers(EventLoop loop) {
        for(SwerveModule module : swerveDrive.getModules()) {
            loop.bind(() -> {
                    if(module.isModuleMisaligned() && !isEnabled()) {
                        new RealignModuleCommand(module).schedule();
                    }
                }
            );  
        }    
    }

    @Override
    protected void determineSelf() {
        if (DriverStation.isEnabled()) {
            if (DriverStation.isTeleop()) {
              setState(State.TeleopFieldOriented);
            }
            else {
              setState(State.Idle);
            }
          }
          else {
            setState(State.Idle);
          }
    }

    @Override
    protected void update() {
        swerveDrive.updateOdometry();
    }

    

    @Override
    public Map<String, Sendable> additionalSendables() {
        return Map.of(
            // "field", drive.getField()
            "module-1", swerveDrive.getModules().get(0),
            "module-2", swerveDrive.getModules().get(1),
            "module-3", swerveDrive.getModules().get(2),
            "module-4", swerveDrive.getModules().get(3)
        );
    }



    public enum State {
        Undetermined,
        Idle,
        TeleopFieldOriented,
        TeleopBotOriented,
        XShape
    }
}
