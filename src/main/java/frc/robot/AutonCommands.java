
package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

// imports

public class AutonCommands {
    private final HashMap<String, Command> kCommandHashMap;
    private final ArrayList<Command> kQueuedCommands;

    public AutonCommands() 
    {
        kCommandHashMap = new HashMap<String, Command>();
        kQueuedCommands = new ArrayList<Command>();
    }

}

// crazy coding skills
/*
 * the drive subsystem does the actual trajectory followoing
 * in choreolib so we need carter to help on this
 * 
 * then uh theres stuff that needs to be done in robot container i think
 * im not sure choreo docs arent very good ok
 * 
 * then we can get to writing the actual file and use
 * uhh commented out code for everytime we need to use a subsystem
 * like //shooter.shoot(values);
 * 
 * man i just wanted to start coding very sad...
 */