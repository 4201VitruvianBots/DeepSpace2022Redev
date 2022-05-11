package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    private final Elevator m_elevator = new Elevator();
    private final Wrist m_wrist = new Wrist();
    private final Climber m_climber = new Climber();

    static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
    static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
    static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);
  
    public Button[] leftButtons = new Button[2];
    public Button[] rightButtons = new Button[2];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[4];
    public Button xBoxLeftTrigger, xBoxRightTrigger;

    public RobotContainer() {
        initializeSubsystems();
        configureButtonBindings();  
    }

    private void configureButtonBindings() {
        for (int i = 0; i < leftButtons.length; i++)
          leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
        for (int i = 0; i < rightButtons.length; i++)
          rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
        for (int i = 0; i < xBoxButtons.length; i++)
          xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
        for (int i = 0; i < xBoxPOVButtons.length; i++)
          xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 90));
    }

    public void initializeSubsystems() {
         m_elevator.setDefaultCommand(new UpdateElevatorSetpoint());
         m_wrist.setDefaultCommand(new UpdateWristSetpoint());
    }
}
