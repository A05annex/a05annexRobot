package org.a05annex.frc;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.commands.AutonomousPathCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.geo2d.KochanekBartelsSpline;

import java.io.FileNotFoundException;

public abstract class A05RobotContainer {

    // declare NavX, used for resetting initial heading
    protected NavX m_navx = NavX.getInstance();

    protected DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    protected A05DriveCommand m_driveCommand;

    protected final XboxController m_driveXbox = new XboxController(A05Constants.DRIVE_XBOX_PORT);

    protected Command m_autoCommand = null;

    public A05RobotContainer() {
        m_driveCommand = new A05DriveCommand(m_driveXbox);
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
            SmartDashboard.putString("Autonomous", String.format("Could not load path: %s", autonomousPath.getName()));
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
