package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public class Sequence implements Runnable {

    private Action action;
    private Sequence sequence;

    public interface Action {
        DThread runAction(); //Return null if not setting anything to a position, return the thread you are waiting on otherwise.
    }

    public Sequence(Action action, Sequence sequence){
        this.action = action;
        this.sequence = sequence;
    }

    public Sequence(Action action){
        this.action = action;
    }

    public void run(){
        if(sequence != null) {
            sequence.run();
        }
        DThread t = action.runAction();
        while(t != null && t.isAlive()){} //Accounts both for threads and single actions.
    }
}
