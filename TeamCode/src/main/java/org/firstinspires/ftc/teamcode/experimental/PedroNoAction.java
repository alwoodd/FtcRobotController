package org.firstinspires.ftc.teamcode.experimental;

public class PedroNoAction implements PedroAction{
    private final String description;

    /**
     * This PedroAction serves as a do-nothing.
     */
    public PedroNoAction() {
        this.description = "";
    }

    public PedroNoAction(String description) {
        this.description = description;
    }

    @Override
    public void update() {
        //Do Nothing
    }

    @Override
    public boolean isComplete() {
        return true;
    }

    @Override
    public String getDescription() {
        return this.description;
    }
}
