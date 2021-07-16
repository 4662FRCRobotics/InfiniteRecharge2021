/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.libraries.ConsoleCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants.ButtonMappings;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive m_drive = new Drive();
  //private final Autonomous m_autonomous = new Autonomous();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();
  public final Vision m_vision = new Vision();
  private final Intake m_intake = new Intake();
  private final ConsoleCommand m_consoleCommand = new ConsoleCommand();

  private final Joystick m_driveStick = new Joystick(0);
  private final Joystick m_console = new Joystick(1);
  
  private final Climb m_climb = new Climb();
  
  //private final CommandBase m_AutoCmd = new StartAutoCmd(m_autonomous, m_drive, m_intake, m_shooter, m_hopper, m_vision, () -> m_stationConsole.getPOV(0),() -> m_stationConsole.getPOV(1));
  private final CommandBase m_AutoCmd = null;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();

    //m_driveStick = new Joystick(0);

    m_drive.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      new ArcadeDrive(
        m_drive,
        () -> m_driveStick.getY(),
        () -> m_driveStick.getTwist(),
        () -> m_driveStick.getThrottle(),
        () -> m_console.getX()));

      
    m_vision.setDefaultCommand(
      new FwdCameraTilt(m_vision,
        () -> m_console.getY()
      )
    );

    m_hopper.retractBeltFrame();
    m_intake.ArmUp();

    //create the auto commands
    // note that building path following during auto is significant elapsed time
    // possibly use a map (lob) array to store commands

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
          new ShootPowerCells(m_shooter, m_driveStick),
          new SequentialCommandGroup(new Wait(1), new ShootHopperFeed(m_hopper)),
          new VisionLightOn(m_vision)
        )
      );

    new JoystickButton(m_driveStick, ButtonMappings.kCLIMB_UP)
    .whileHeld(
      new ConditionalCommand(
        new ClimbUp(m_climb),
        new InstantCommand(),
        () -> m_driveStick.getRawButton(ButtonMappings.kCLIMB_SWITCH))
      );
    
    new JoystickButton(m_driveStick, ButtonMappings.kCLIMB_DOWN).whileHeld(
      new ClimbDown(m_climb));
    
    //new JoystickButton(m_driveStick, ButtonMappings.kCLIMB_SWITCH)
    //.whenPressed(() -> m_vision.setServoUp())
    //.whenReleased(() -> m_vision.setServoDown());
  
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
      new CloseLoader(m_hopper, m_intake)
    );

    new JoystickButton(m_driveStick, ButtonMappings.kVISION_ON).whileHeld(
      new VisionLightOn(m_vision));
  }

  public void getAutonomousName() {
    // An ExampleCommand will run in autonomous
    // lamda for pov - "() -> m_Console.getPOV(0)"
    String commandName = m_consoleCommand.getPatternName(() -> m_console.getPOV(0), () -> m_console.getPOV(1));
    SmartDashboard.putString("Auto Name", commandName);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // use the station console auto switches to determine the commmand to execute
    String commandName = m_consoleCommand.getPatternName(() -> m_console.getPOV(0), () -> m_console.getPOV(1));
    SmartDashboard.putString("Auto Name", commandName);
    Command autoCommand = m_consoleCommand.getSelected(() -> m_console.getPOV(0), () -> m_console.getPOV(1));
    Boolean bIsCommandFound = autoCommand != null;
    SmartDashboard.putBoolean("Auto Found", bIsCommandFound);
    return autoCommand;
  }
}
