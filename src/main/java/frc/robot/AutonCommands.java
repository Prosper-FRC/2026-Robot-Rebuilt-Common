
package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// imports

public class AutonCommands {

    public static HashMap<String, Command> commandHashMap;
    private final ArrayList<Command> queuedCommands;


    public AutonCommands() 
    {
        commandHashMap = new HashMap<String, Command>();
        queuedCommands = new ArrayList<Command>();
    }

    public Command getPath(String key) 
    {
        return commandHashMap.get(key);
    }

    public Command getPath(String key, double wait) 
    {
        return commandHashMap.get(key);
    }

    public boolean isPathComplete() 
    {
        return false;
    }

    // Implement later
    public Command runQueue() 
    { 
        return new SequentialCommandGroup(); 
    }

    // This may change, depends on the will of king carter (swervesample?)
    public Pose2d getPathPose() // take timestamp
    {
        return new Pose2d();
    }

    public ChassisSpeeds getPathSpeeds() // take timestamp
    {
        return new ChassisSpeeds();
    }

    //optional thingy idk i dont understand it

    public void addCustomCommand(Command command)
    {
        queuedCommands.add(command);
    }
}

// crazy coding skills
/*
 * ok so a lot of stuff needs to be actually implemented but im hungry
 * also ragav i have no flippity flopping idea why the choreo folders are seperate ok
 * please delete the folder on your side or something idk i can fix this hopefully
 */