/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GetNextAutoCmd extends SequentialCommandGroup {
  Drive m_drive;
  Autonomous m_autonomous;
  Intake m_intake;
  Shooter m_shooter;
  Hopper m_hopper;
  Vision m_vision;

  
  /**
   * Creates a new GetNextAutoCmd.
   */

  public GetNextAutoCmd(Autonomous autonomous, Drive drive, Intake intake, Shooter shooter,  Hopper hopper, Vision vision)  {
    
    m_drive = drive;
    m_autonomous = autonomous;
    m_intake = intake;
    m_shooter = shooter;
    m_hopper = hopper;
    m_vision = vision;


    String command = "";

    do {
      command = autonomous.getNextCmd();
      ProcessCommand(command);
    } while (!autonomous.isFinished());


    if(command != ""){
      addCommands(new StartGetNextCmd(m_autonomous, m_drive, m_intake, m_hopper, m_shooter, m_vision));
    }else{
      System.out.println("exiting command loop");
    }
    
  }
  private void ProcessCommand(final String command){
    double time;
    System.out.println("Scheduled Command: " + command);
    switch (command) {
      case "wait":
        time = m_autonomous.getDoubleCommandValue();
        //System.out.println("Wait Command is waiting for " + time + " seconds.");
        addCommands(new Wait(time));
        break;
      
      case "driveDistance":
        final double distance = m_autonomous.getDoubleCommandValue();
        final double setPoint = -distance * DriveConstants.kPULSE_PER_ROTATION * DriveConstants.kGEARBOX_REDUCTION / (DriveConstants.kTIRE_SIZE * Math.PI);
        //System.out.println("Drive Distance is driving for " + distance + " inchs.");
        //addCommands(new DriveDistance(distance, m_drive));
        addCommands(new DriveDistance(setPoint, m_drive));
        /*System.out.println("Pulse per rotation:" + DriveConstants.kPULSE_PER_ROTATION);
        System.out.println("Reduction of the gearboxes:" + DriveConstants.kGEARBOX_REDUCTION);
        System.out.println("Tire Diameter:" + DriveConstants.kTIRE_SIZE);
        System.out.println("PI is:" + Math.PI);
        System.out.println("Distance travel is:" + setPoint + " inchs(?//Theocatically)");
        */
        break;

      case "turnToAngle":
        //System.out.println("Turn angle value: " + angle);
        final double angle = m_autonomous.getDoubleCommandValue();
        //System.out.println("Turn Angle is turning for " + angle + " degrees.");
        addCommands(new TurnToAngle(angle, m_drive));
        break;

      case "combineDown":
        //System.out.println("Combine is coming down.");
        addCommands(new CombineDown(m_intake));
        break;  

      /*case "combineOn":
        System.out.println("Combine is turning on.");
        addCommands(new CombineOn(m_intake));
        break;*/
      
      case "shooter":
        time = m_autonomous.getDoubleCommandValue(); 
        //System.out.println("Shooter is Shooting for: " + time + " Seconds");
        addCommands(new ConditionalCommand(new AutoShoot(time, m_shooter, m_vision, m_drive, m_hopper), new DriveDistance(-48, m_drive), m_vision::isHighGoalAligned));
        break;   

      case "":
        //System.out.println("No command is commanding.");
        break;

      default:
        //System.out.println("Unrecognized command: " + command);
    }
  }
}
