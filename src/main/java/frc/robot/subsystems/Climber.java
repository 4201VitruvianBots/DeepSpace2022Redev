/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climber extends SubsystemBase {
    private final Elevator m_elevator;
    private final Wrist m_wrist;

    public TalonSRX master = new TalonSRX(Constants.Climber.climbMotor);

    DoubleSolenoid climbPistons = new DoubleSolenoid(
        Constants.Pneumatics.pcmOne, 
        Constants.Pneumatics.pcmType,
        Constants.Pneumatics.climbPistonForward, 
        Constants.Pneumatics.climbPistonReverse);

    public Climber(Elevator elevator, Wrist wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
    }

    public void setClimberOutput(double output){
        master.set(ControlMode.PercentOutput, output);
    }

    public void setClimbPistonState(boolean state) {
        climbPistons.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public boolean getClimbPistonState() {
        return climbPistons.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public boolean isClimbMode() {
        return m_elevator.getControlMode() == 0 && m_wrist.getControlMode() == 0;
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Climb Mode", isClimbMode());
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }
}
