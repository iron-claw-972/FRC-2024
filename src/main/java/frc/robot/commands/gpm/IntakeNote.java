package frc.robot.commands.gpm;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;
import lib.controllers.GameController;
import lib.controllers.PS5Controller;

public class IntakeNote extends Command{

    private final Intake intake;
    private final StorageIndex storageIndex;
    private final Arm arm;
    private Consumer<Boolean> reactor;
    Timer timer = new Timer();
    Boolean detectedNote = false;

// very jank, must add more constructors
    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm, Consumer<Boolean> reactor) {
        this.intake = intake;
        this.storageIndex = storageIndex;
        this.arm = arm;
        this.reactor = reactor;

        addRequirements(intake, storageIndex, arm);
        // addRequirements(intake, storageIndex);

    }
    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm) {
        this.intake = intake;
        this.storageIndex = storageIndex;
        this.arm = arm;
        this.reactor = (x)->{
            };

        addRequirements(intake, storageIndex, arm);

    }
    public IntakeNote(Intake intake, StorageIndex index, Consumer<Boolean> reactor) {
        this.intake = intake;
        this.storageIndex = index;
        this.arm = null;
        this.reactor = reactor;
        addRequirements(intake, index);
    }

    @Override
    public void initialize() {
        detectedNote = false;
        timer.reset();
        timer.stop();
        intake.setMode(Mode.INTAKE);
        storageIndex.runIndex();
        arm.setAngle(ArmConstants.intakeSetpoint);
    }

    @Override
    public void execute(){
        storageIndex.runIndex();
        if (intake.hasNote()){
            detectedNote = true;
        }
        if(!intake.hasNote() && detectedNote){
            timer.start();
            reactor.accept(true);
        }
    }

    @Override
    public boolean isFinished(){
        if (timer.hasElapsed(0.01)) {
            intake.setMode(Mode.DISABLED);
            storageIndex.stopIndex();
            detectedNote = false;
            //arm.setAngle(ArmConstants.stowedSetpoint);
        }
        return timer.hasElapsed(1); 
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted) {
            intake.setMode(Mode.DISABLED);
            storageIndex.stopIndex();
            detectedNote = false;
            arm.setAngle(ArmConstants.stowedSetpoint);
        }
        reactor.accept(false);
        timer.stop();
        timer.reset();
    }
    
}
