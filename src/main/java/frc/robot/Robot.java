package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;

  private final EventLoop checkModulesLoop = new EventLoop();

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer(checkModulesLoop);

    SubsystemManagerFactory.getInstance().registerSubsystem(robotContainer);
    SubsystemManagerFactory.getInstance().disableAllSubsystems();

    //Update the event loop for misaligned modules once every 10 seconds
    addPeriodic(checkModulesLoop::poll, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    SubsystemManagerFactory.getInstance().disableAllSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SubsystemManagerFactory.getInstance().notifyAutonomousStart();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    SubsystemManagerFactory.getInstance().notifyTeleopStart();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
