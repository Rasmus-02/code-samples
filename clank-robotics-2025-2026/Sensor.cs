using Godot;
using System;
using System.Collections.Generic;

public abstract partial class Sensor : Placeable, Block
{
    // Variables
    [Export] public float SensorRange = 10; // in meters
    [Export] public int RefreshRate = 10; // update per second
    private int UpdateTick = 0;
    private int currentUpdateTick = 0;

    // Classes to implement:
    public abstract object GetSensorData();
    public abstract void ScanTarget();
    public abstract void instantiate();

    public override void _Ready()
    {
        UpdateTick = 60 / RefreshRate;
        instantiate(); // Used as "Ready" for derived classes
    }

    public override void _PhysicsProcess(double delta)
    {
        currentUpdateTick += 1;
        if (currentUpdateTick >= UpdateTick)
        {
            currentUpdateTick = 0;
            ScanTarget();
        }
    }

    
}
