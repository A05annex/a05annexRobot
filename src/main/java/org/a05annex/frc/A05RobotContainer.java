package org.a05annex.frc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.commands.AutonomousPathCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;

import java.io.FileNotFoundException;

/**
 * This is the abstract class for the RobotContainer which provides the boilerplate
 * swerve drive, NavX, camera, driver settings, robot settings, and autonomous
 * path configuration for the robot.
 */
public abstract class A05RobotContainer {

    /**
     * Theis is the {@link NavX} that provides inertial navigation for the robot.
     */
    protected NavX m_navx = NavX.getInstance();

    /**
     * This is the swerve drive subsystem.
     */
    protected DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    /**
     * This is the default drive command for this year's competition.
     */
    protected A05DriveCommand m_driveCommand;

    /**
     * This is the robot driver XBox controller.
     */
    protected final XboxController m_driveXbox = new XboxController(A05Constants.DRIVE_XBOX_PORT);

    /**
     * This is the autonomous path following command initialized for the path specified
     * by the autonomous path selection switches.
     */
    protected AutonomousPathCommand m_autoCommand = null;

    /**
     * These are the driver-specific controller settings (gain, sensitivity, deadband, etc.) for
     * the driver selected by the driver selection switches.
     */
    protected A05Constants.DriverSettings m_driver = null;

    /**
     * This is the robot-specific description of this robot geometry, drive  initialization,
     * and speed calibration correction.
     */
    protected A05Constants.RobotSettings m_robotSettings = null;

    /**
     * The default robot container initialization, which:
     * <ul>
     *     <li>Reads the driver selection switches and sets the driver;</li>
     *     <li>Reads the robot Id switch and sets the robot between the
     *         <i>programming</i> and <i>competition robots</i>;</li>
     *     <li>Reads the autonomous selection switches, loads the specified
     *         autonomous path and initializes the {@link #m_autoCommand}</li>
     *     <li>Starts the USB camera if one exists.</li>
     * </ul>
     */
    public A05RobotContainer() {
        int driverId = A05Constants.readDriverID();
        try {
            m_driver = A05Constants.DRIVER_SETTINGS_LIST.get(driverId);
            m_driver.load();
            m_driveCommand = new A05DriveCommand(m_driveXbox, m_driver);
            SmartDashboard.putString("Driver", m_driver.getName());
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putString("Driver", String.format("Driver ID %d does not exist", driverId));
            throw e;
        } catch (RuntimeException e) {
            SmartDashboard.putString("Driver",
                    String.format("Could not load driver: '%s'", m_driver.getName()));
            throw e;
        }

        int robotId = A05Constants.readRobotID();
        m_robotSettings = A05Constants.ROBOT_SETTINGS_LIST.get(robotId);

        // autonomous
        int autoId = A05Constants.readAutoID();
        A05Constants.AutonomousPath autonomousPath = null;
        try {
            autonomousPath = A05Constants.AUTONOMOUS_PATH_LIST.get(autoId);
            autonomousPath.load();
            m_autoCommand = new AutonomousPathCommand(autonomousPath, m_driveSubsystem);
            SmartDashboard.putString("Autonomous", autonomousPath.getName());
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putString("Autonomous", String.format("Path ID %d does not exist", autoId));
        } catch (FileNotFoundException e) {
            SmartDashboard.putString("Autonomous",
                    String.format("Could not load path: '%s'", autonomousPath.getName()));
        }

        if (A05Constants.hasUsbCamera()) {
            // Start logitech camera
            CameraServer.startAutomaticCapture();
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link A05Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }
}
