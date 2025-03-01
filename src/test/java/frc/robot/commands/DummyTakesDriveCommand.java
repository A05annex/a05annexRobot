package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.a05annex.frc.commands.ICanTakeDrive;

public class DummyTakesDriveCommand extends Command implements ICanTakeDrive {

    public static int instantiationCt = 0;
    public static int initializationCt = 0;
    public static int endCt = 0;
    public static int executeCt = 0;

    final int cyclesBeforeTakeDrive;
    final int cyclesToTarget;

    int cycleCt = 0;

    public DummyTakesDriveCommand() {
        this.cyclesBeforeTakeDrive = 0;
        this.cyclesToTarget = 50;
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        synchronized (DummyTakesDriveCommand.class) {
            instantiationCt += 1;
        }
    }
    public DummyTakesDriveCommand(Integer cyclesBeforeTakeDrive, Integer cyclesToTarget) {
        this.cyclesBeforeTakeDrive = cyclesBeforeTakeDrive;
        this.cyclesToTarget = cyclesToTarget;
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        synchronized (DummyTakesDriveCommand.class) {
            instantiationCt += 1;
        }
    }
    @Override
    public boolean canTakeDrive() {
        if (cycleCt < cyclesBeforeTakeDrive) {
            cycleCt++;
            System.out.printf("false = %s.canTakeDrive(), ct = %d%n", this.getClass().getName(), cycleCt);
            return false;
        }
        System.out.printf("true = %s.canTakeDrive()%n", this.getClass().getName());
        return true;
    }
    @Override
    public void initialize() {
        synchronized (DummyTakesDriveCommand.class) {
            initializationCt += 1;
        }

    }
    @Override
    public void execute() {
        synchronized (DummyTakesDriveCommand.class) {
            executeCt += 1;
        }
        cycleCt++;
        System.out.printf("%s.execute(), ct = %d%n", this.getClass().getName(), cycleCt);
    }

    @Override
    public boolean isFinished() {
        return cycleCt >= cyclesToTarget;
    }

    @Override
    public void end(boolean interrupted) {
        synchronized (DummyTakesDriveCommand.class) {
            endCt++;
        }
    }

    /**
     * return true so we can test this thing.
     * @return {@code true}
     */
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
