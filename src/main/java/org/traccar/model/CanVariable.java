package org.traccar.model;

public class CanVariable {

    //region getCanVariable, variables from xml MARX
    public CanVariable(){

    }

    private String varId;

    public String getVarId(){
        return this.varId;
    }

    public void setVarId(String varId){
        this.varId = varId;
    }

    private String title;

    public String getTitle(){
        return this.title;
    }

    public void setTitle(String title){
        this.title = title;
    }

    private int fwMultiplier;

    public int getFwMultiplier(){
        return this.fwMultiplier;
    }

    public void setFwMultiplier(int fwMultiplier){
        this.fwMultiplier = fwMultiplier;
    }

    private int fwDivider;

    public int getFwDivider(){
        return this.fwDivider;
    }

    public void setFwDivider(int fwDivider){
        this.fwDivider = fwDivider;
    }

    private int fwOffset;

    public int getFwOffset(){
        return this.fwOffset;
    }

    public void setFwOffset(int fwOffset){
        this.fwOffset = fwOffset;
    }

    private String fwUnits;

    public String getFwUnits(){
        return this.fwUnits;
    }

    public void setFwUnits(String fwUnits){
        this.fwUnits = fwUnits;
    }

    private int decimalPlaces;

    public int getDecimalPlaces(){
        return this.decimalPlaces;
    }

    public void setDecimalPlaces(int decimalPlaces){
        this.decimalPlaces = decimalPlaces;
    }

    //endregion
}
