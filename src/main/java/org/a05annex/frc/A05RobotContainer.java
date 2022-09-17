package org.a05annex.frc;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;

public abstract class A05RobotContainer
{

    // declare NavX, used for resetting initial heading
    protected NavX m_navx = NavX.getInstance();

    protected DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    protected A05DriveCommand m_driveCommand;

    protected final XboxController m_driveXbox = new XboxController(A05Constants.DRIVE_XBOX_PORT);

    public A05RobotContainer() {
        m_driveCommand = new A05DriveCommand(m_driveXbox);
    }
}
