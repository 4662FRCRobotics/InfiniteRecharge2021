/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.libraries.AutoPosition;
import frc.robot.libraries.AutonomousCommand;
import frc.robot.libraries.ConsoleJoystick;
import frc.robot.subsystems.*;
import frc.robot.Constants.ButtonMappings;
import frc.robot.Constants.ConsoleCommandConstants;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive m_drive = new Drive();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();
  public final Vision m_vision = new Vision();
  private final Intake m_intake = new Intake();
  private final Climb m_climb = new Climb();
 
  private final Joystick m_driveStick = new Joystick(0);
  private final ConsoleJoystick m_console = new ConsoleJoystick(1);
  
  public final Command m_autoCmd = new Autonomous1(m_console, m_drive, m_hopper, m_intake, m_shooter, m_vision);
  private final AutonomousCommand m_autoCommand = new AutonomousCommand();
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new ArcadeDrive(
        m_drive,
        () -> m_driveStick.getY(),
        () -> m_driveStick.getTwist(),
        () -> m_driveStick.getThrottle(),
        () -> m_console.getX()
      )
    );
      
    m_vision.setDefaultCommand(
      new FwdCameraTilt(m_vision,
        () -> m_console.getY()
      )
    );

    m_shooter.setDefaultCommand(
      new ShootSetPower(m_shooter,
      () -> m_console.getZ(),
      () -> m_console.getThrottle()
      )
    );

    m_hopper.retractBeltFrame();
    m_intake.ArmUp();

    //create the auto commands
    // note that building path following during auto is significant elapsed time
    // possibly use a map (lob) array to store commands
    m_autoCommand.setDefaultOption(AutoPosition.MIDDLE.name(), new AutoControl(AutoPosition.MIDDLE));
    m_autoCommand.addOption(AutoPosition.LEFT.name(), new AutoControl(AutoPosition.LEFT));
    m_autoCommand.addOption(AutoPosition.RIGHT.name(), new AutoControl(AutoPosition.RIGHT));

  }

  

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
    new JoystickButton(m_driveStick, ButtonMappings.kSHOOTER)
    .whileHeld(
      new ParallelCommandGroup(
        new ShootPowerCells(m_shooter, () -> m_console.getZ(), () -> m_console.getThrottle()),
        new SequentialCommandGroup(new Wait(1), new ShootHopperFeed(m_hopper))
//          new VisionLightOn(m_vision)
      )
    );

    // conditional command requires a true on the second attribute for the command to run
    // this is a "deadman" two button to raise the climber - reduce risk of accidental lift before intended
    new JoystickButton(m_driveStick, ButtonMappings.kCLIMB_UP)
    .whileHeld(
      new ConditionalCommand(
        new ClimbUp(m_climb),
        new InstantCommand(),
        () -> m_console.getRawButton(ButtonMappings.kCLIMB_SWITCH)
      )
    );
    
    new JoystickButton(m_driveStick, ButtonMappings.kCLIMB_DOWN).whileHeld(
      new ClimbDown(m_climb));
     
    new  JoystickButton(m_driveStick, ButtonMappings.kLOADER)
    .whileHeld(
      new HarvestPowerCells(m_hopper, m_intake)
    );

    new JoystickButton(m_driveStick, ButtonMappings.kLOADERSPIT)
    .whileHeld(
      new SpitPowerCells(m_intake)
    );

    new JoystickButton(m_driveStick, ButtonMappings.kCLOSELOADER)
    .whenPressed(
      new RetractHarvester(m_hopper, m_intake)
    );

    new JoystickButton(m_driveStick, ButtonMappings.kVISION_ON).whileHeld(
      new VisionLightOn(m_vision));

    // testing auto drive distance for PID tuning
    SmartDashboard.putData("AutoDistance", new AutoDriveDistance(m_drive));
  }

    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void checkAutonomousSetting() {
    int iPosition = m_console.getROT_SW_0();
    String patternName = "";
    boolean bIsCommandFound = false;
    if ( iPosition < ConsoleCommandConstants.kPOS_PATTERN_NAME.length) {
      patternName = ConsoleCommandConstants.kPOS_PATTERN_NAME[iPosition];
      bIsCommandFound = true;
    }
    String patternName2 = AutoPosition.getByIndex(iPosition).name();
    
    SmartDashboard.putString("Auto Name", patternName + patternName2);
    SmartDashboard.putBoolean("Auto Found", bIsCommandFound);
    SmartDashboard.putNumber("Auto Delay", m_console.getROT_SW_1());
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // use the station console auto switches to determine the commmand to execute
    return m_autoCmd;
  }
}
