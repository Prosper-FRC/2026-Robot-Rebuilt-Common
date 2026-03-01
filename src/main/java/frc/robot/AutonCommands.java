
package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// imports

public class AutonCommands {

    private final HashMap<String, Command> kCommandHashMap;
    private final ArrayList<Command> kQueuedCommands;

    // We might need these?
    private SendableChooser<Command> autoChooser;

    /* someone please explain why we put m
     * private Drive robotdrive;
     * private Shooter mShooter;
     * private Intake mIntake;
     * private Indexer mIndexer;
     * private Superstructure robotSuperstructure;
     * 
     */


    public AutonCommands() 
    {
        kCommandHashMap = new HashMap<String, Command>();
        kQueuedCommands = new ArrayList<Command>();

        autoChooser = new SendableChooser<>();
    }

    public void addCommandToQueue(String name) 
    {
        kQueuedCommands.add(kCommandHashMap.get(name));
    }

    public void runQueue()
    {
        
    }

    // Implement later
    private Command getQueueAsSequentialCommand(ArrayList<Command> queue) { return new SequentialCommandGroup(null); }

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