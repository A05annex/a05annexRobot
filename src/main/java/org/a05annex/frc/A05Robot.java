package org.a05annex.frc;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;

/**
 * This is the basic A05annex Robot that does All the common stuff for a swerve drive base with NavX mounted
 * under the base of the robot, and switches for autonomous program and driver selection. This should free
 * the programmers to only worry about the attachments specific to the competition.
 */
public abstract class A05Robot extends TimedRobot {

    /**
     * The a05RobotContainerInstance instance
     */
    protected A05RobotContainer a05RobotContainer;

    private Command autonomousCommand = null;

    /**
     * Set the robot container, which must be a superclass of {@link A05RobotContainer}, for this robot.
     *
     * @param container The robot container.
     */
    @SuppressWarnings("unused")
    protected void setRobotContainer(A05RobotContainer container) {
        a05RobotContainer = container;
    }

    @Override
    public void robotInit() {
        // Starts recording to data log
        DataLogManager.start();

        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    /**
     * This method is called every robot command cycle, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        PhotonCameraWrapper.updateAllTrackingData(); // This is happening above the command scheduler to ensure there is
                                                     // fresh data for subsystems and commands to run on

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

    }

    @Override
    public void teleopInit()
    {
        DriveSubsystem.getInstance().calibrate();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }

    }

    /**
     *  This is the initialization at the start of the autonomous period. If an autonomous command is obtained from
     *  the {@link A05RobotContainer}, then is scheduled to be run in autonomous. */
    @Override
    public void autonomousInit()
    {
        DriveSubsystem.getInstance().calibrate();

        autonomousCommand = a05RobotContainer.getAutonomousCommand();
        // if there is an autonomous command, schedule it.
        if (autonomousCommand != null)
        {
            // Autonomous paths ALWAYS assume FIELD_RELATIVE
            DriveSubsystem.getInstance().setDriveMode(ISwerveDrive.DriveMode.FIELD_RELATIVE);
            autonomousCommand.schedule();
            if (A05Constants.getPrintDebug()) {
                System.out.println("A05Robot.autonomousInit scheduled autonomousCommand");
            }
        }
    }
}
