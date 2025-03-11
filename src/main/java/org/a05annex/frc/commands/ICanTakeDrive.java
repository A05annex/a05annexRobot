package org.a05annex.frc.commands;

/**
 * This is an interface that must be implemented in a command that the {@link AutonomousPathCommand}
 * may relinquish drive control to. After the command implementing this interface is instantiated, the
 * {@link AutonomousPathCommand} tries to relinquish drive control, but, will only do so when
 * the {@link #canTakeDrive()} method returns {@code true}.
 *<p>
 * Once the command implementing this interface says it can take control, the {@link AutonomousPathCommand#execute()}
 * method will defer drive control to the implementing command until it finishes. If, for some reason, the
 * implementing command cannot assume control, then the {@link AutonomousPathCommand} control will follow the
 * programmed path to the next control point, and will resume normal path following after that.
 */
public interface ICanTakeDrive {

    /**
     * Test whether this command is ready to take control of the drive.
     *
     * @return {@code true} if this command has identified its navigation target and is
     * ready to take control of the drive, {@code false} otherwise.
     */
     boolean canTakeDrive();
}
