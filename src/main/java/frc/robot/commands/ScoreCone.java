package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pneumatics;

// Composite command for scoring a cone
public class ScoreCone extends SequentialCommandGroup {
// ((new moveArmPos(arm, highConeGridAngle,highConeGridExtension).raceWith(new WaitCommand(highConeGridTimeout))).andThen(new Grabber(pneumatics))).andThen(new WaitCommand(0.05)).andThen(new moveArmPos(arm, 30, 10)));
   
    public ScoreCone(Arm arm, Pneumatics pneumatics, double armAngle, double armTele, double timeout, double holdTime) {

        addCommands(new moveArmPos(arm, armAngle, armTele).raceWith(new WaitCommand(timeout)),  // rotate and extend the arm to the specified position, for at most the timeout
        new Grabber(pneumatics), // open the grabber
        new WaitCommand(holdTime), // Wait for the grabber to release
        new moveArmPos(arm, 30, 10) // return the arm to a neutral position
        
        );
    
    }
    
}
