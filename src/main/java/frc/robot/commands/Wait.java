package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class Wait extends Command {

    // Variable to store wait time
    private double delay;

    /**
     * Instantiates a new wait command.
     *
     * @param wait
     *            The wait time in seconds
     */
    public Wait(double wait) {
        this.delay = wait;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        setTimeout(delay);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Command is finished once the wait time is over
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}