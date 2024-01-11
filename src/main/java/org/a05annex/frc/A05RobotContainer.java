package org.a05annex.frc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
     * This is the {@link NavX} that provides inertial navigation for the robot.
     */
    protected NavX navx = NavX.getInstance();

    /**
     * This is the swerve drive subsystem.
     */
    protected DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    /**
     * This is the default drive command for this year's competition.
     */
    protected A05DriveCommand driveCommand;

    /**
     * This is the robot driver Xbox controller typically used to control the drive.
     */
    protected final XboxController driveXbox = A05Constants.DRIVE_XBOX;

    /**
     * This is the alternate Xbox controller typically used to control non-drive subsystems.
     */
    protected final XboxController altXbox = A05Constants.ALT_XBOX;
    /**
     * This is the autonomous path following command initialized for the path specified
     * by the autonomous path selection switches.
     */
    protected AutonomousPathCommand autoCommand = null;

    /**
     * This is the robot-specific description of this robot geometry, drive  initialization,
     * and speed calibration correction.
     */
    protected A05Constants.RobotSettings robotSettings = null;

    // drive controller button declarations
    protected JoystickButton driveA = new JoystickButton(driveXbox, 1),
                   driveB = new JoystickButton(driveXbox, 2),
                   driveX = new JoystickButton(driveXbox, 3),
                   driveY = new JoystickButton(driveXbox, 4),
                   driveLeftBumper = new JoystickButton(driveXbox, 5),
                   driveRightBumper = new JoystickButton(driveXbox, 6),
                   driveBack = new JoystickButton(driveXbox, 7),
                   driveStart = new JoystickButton(driveXbox, 8),
                   driveLeftStickPress = new JoystickButton(driveXbox, 9),
                   driveRightStickPress = new JoystickButton(driveXbox, 10);

    // alternate controller button declarations
    protected JoystickButton altA = new JoystickButton(altXbox, 1),
                   altB = new JoystickButton(altXbox, 2),
                   altX = new JoystickButton(altXbox, 3),
                   altXY = new JoystickButton(altXbox, 4),
                   altLeftBumper = new JoystickButton(altXbox, 5),
                   altRightBumper = new JoystickButton(altXbox, 6),
                   altBack = new JoystickButton(altXbox, 7),
                   altStart = new JoystickButton(altXbox, 8),
                   altLeftStickPress = new JoystickButton(altXbox, 9),
                   altRightStickPress = new JoystickButton(altXbox, 10);

    /**
     * The default robot container initialization, which:
     * <ul>
     *     <li>Reads the driver selection switches and sets the driver;</li>
     *     <li>Reads the robot Id switch and sets the robot between the
     *         <i>programming</i> and <i>competition robots</i>;</li>
     *     <li>Reads the autonomous selection switches, loads the specified
     *         autonomous path and initializes the {@link #autoCommand}</li>
     *     <li>Starts the USB camera if one exists.</li>
     * </ul>
     */
    public A05RobotContainer() {
        // setup the driver profile - read the configuration switches 0 and 1 to get the driver id (0 to 3)
        int driverId = A05Constants.readDriverID();
        try {
            // load the driver profile file
            A05Constants.setDriver(A05Constants.DRIVER_SETTINGS_LIST.get(driverId));
            A05Constants.getDriver().load();
            driveCommand = new A05DriveCommand(DriveSubsystem.getInstance());
            SmartDashboard.putString("Driver", A05Constants.getDriver().getName());
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putString("Driver", String.format("Driver ID %d does not exist", driverId));
            throw e;
        } catch (RuntimeException e) {
            SmartDashboard.putString("Driver",
                    String.format("Could not load driver: '%s'", A05Constants.getDriver().getName()));
            throw e;
        }

        // Which robot is this? competition or spare/prototype
        int robotId = A05Constants.readRobotID();
        robotSettings = A05Constants.ROBOT_SETTINGS_LIST.get(robotId);

        // setup the chosen autonomous path
        int autoId = A05Constants.readAutoID();
        A05Constants.AutonomousPath autonomousPath = null;
        try {
            autonomousPath = A05Constants.AUTONOMOUS_PATH_LIST.get(autoId);
            autonomousPath.load();
            autoCommand = new AutonomousPathCommand(autonomousPath, driveSubsystem);
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
        return autoCommand;
    }
}
