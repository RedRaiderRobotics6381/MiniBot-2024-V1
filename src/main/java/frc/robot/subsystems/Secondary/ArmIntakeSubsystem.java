package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmIntakeSubsystem extends SubsystemBase {

    public CANSparkMax intakeMotorMaster;
    public CANSparkMax intakeMotorFollower;

    public ArmIntakeSubsystem() {
        intakeMotorMaster =  new CANSparkMax(Constants.ArmConstants.kManipulatorIntakeMotorL, MotorType.kBrushless);
        intakeMotorFollower =  new CANSparkMax(Constants.ArmConstants.kManipulatorIntakeMotorR, MotorType.kBrushless);
        intakeMotorFollower.follow(intakeMotorMaster, true);
    }
    
    public Command ArmIntakeCmd(double ArmIntakeSetpoint) {
        // implicitly require `this`
        return this.runOnce(() -> intakeMotorMaster.set(ArmIntakeSetpoint));
        
        //armSubsystem.intakeMotorR.set(Constants.ArmConstants.gIntakeSpeed););
    }

}