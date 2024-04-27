package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  @AutoLogOutput private Pose3d botPose = new Pose3d();

  @AutoLogOutput private Pose3d[] componentPoses = new Pose3d[0];
  @AutoLogOutput private Pose2d botPose2D = new Pose2d();

  private final EventLoop checkModulesLoop = new EventLoop();

  private int currentLoops = 0;

  @Override
  public void robotInit() {

    if (isReal()) Constants.currentBuildMode = BuildMode.REAL;

    PowerDistribution powerDist = null;

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.currentBuildMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        powerDist = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        Logger.setReplaySource(null);
        break;
      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
    }

    robotContainer = new RobotContainer(checkModulesLoop, powerDist);

    SubsystemManagerFactory.getInstance().registerSubsystem(robotContainer);
    SubsystemManagerFactory.getInstance().disableAllSubsystems();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    updatePoses();

    // check the modules loop ~ once every 10 seconds
    if (currentLoops++ > 500) {
      currentLoops = 0;
      checkModulesLoop.poll();
    }
  }

  /** Update the poses of the bot and the various components for advantage scope viewing */
  private void updatePoses() {
    botPose = robotContainer.getBotPose();

    botPose2D = botPose.toPose2d();
    componentPoses = new Pose3d[] {};
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
