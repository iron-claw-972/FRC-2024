package frc.robot.commands.gpm;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;

public class IntakeNote extends Command{

    private final Intake intake;
    private final StorageIndex storageIndex;
    private final Arm arm;

    private final Consumer<Boolean> rumbleConsumer;

    private final Timer timer = new Timer();
    private Boolean detectedNote = false;

    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm, Consumer<Boolean> rumbleConsumer) {
        this.intake = intake;
        this.storageIndex = storageIndex;
        this.arm = arm;
        this.rumbleConsumer = rumbleConsumer;
        addRequirements(intake, storageIndex, arm);
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
        }
        rumbleConsumer.accept(detectedNote);
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(0.01); 
    }

    @Override
    public void end(boolean interupted){
        if(detectedNote){
            CommandScheduler.getInstance().schedule(new RunCommand(() -> {
                rumbleConsumer.accept(true);
            }).withTimeout(0.5).andThen(new InstantCommand(()->{
                rumbleConsumer.accept(false);
            })));
        }
        intake.setMode(Mode.DISABLED);
        storageIndex.stopIndex();
        arm.setAngle(ArmConstants.stowedSetpoint);
        timer.stop();
        timer.reset();
        detectedNote = false;
    }
    
}
