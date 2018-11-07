package org.firstinspires.ftc.teamcode.subsystems.team_marker;

/**
 * Created by Sarthak on 9/12/2018.
 */
public interface ITeamMarker {
    /**
     * Drops the team marker, releases it from the robot
     */
    void drop();

    /**
     * Holds the team marker in a secure position to keep it in the robot
     */
    void hold();
}
