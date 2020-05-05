package frc.robot.commands.autonomous;

public enum AutoBrakeMode {
    // TODO add UNSET option to keep brake mode as is
    /**
     * No brake at all
     */
    NONE,
    /**
     * Set to brake at start and don't set it back to coast
     */
    ALWAYS,
    /**
     * Set brake to true at start and then to false at end
     */
    WHILE_RUNNING_INSTANT,
    /**
     * Set brake to true at start,
     * when the command ends wait for a fixed amount of time and then set to false
     */
    WHILE_RUNNING_DELAY;
}