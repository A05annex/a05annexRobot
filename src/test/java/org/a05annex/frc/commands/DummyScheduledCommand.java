package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DummyScheduledCommand extends CommandBase {

    public DummyScheduledCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
