using Godot;
using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

public partial class RlAgent : CharacterBody3D
{
	public const float Speed = 5.0f;
	public const float JumpVelocity = 4.5f;
	[Export] Marker3D respawnMarker;
    private Node3D aiController;
	float progress = 0.0f;
    float last_progress = 0.0f;
    float bestProgress = 0.0f;
    float timer = 0.0f;
    bool cheated = false;

    Path3D track;


    public override void _Ready()
    {
        track = GetParent().GetNode<Path3D>("Path3D");
        aiController = (Node3D)GetNode("AIController3D");
    }

    public override void _PhysicsProcess(double delta)
	{
        cheated = false;
        Vector3 velocity = Velocity;

        // Add the gravity.
        if (!IsOnFloor())
		{
			velocity += GetGravity() * (float)delta;
		}

		Vector2 move = (Vector2)aiController.Get("move");
		//Vector2 move = Input.GetVector("ui_left", "ui_right", "ui_up", "ui_down");
        velocity.X = move.X;
		velocity.Z = move.Y;


        Velocity = velocity;
		MoveAndSlide();

		timer += (float)delta;
		if (timer > 240)
            Reset();

        float closestOffset = track.Curve.GetClosestOffset(GlobalPosition);
        float totalLenght = track.Curve.GetBakedLength();
        progress = closestOffset / totalLenght;

        // Punnish if cheating
        if (last_progress <= 0.1 && progress >= 0.9f)
        {
            Cheat(this, 1.0f);
        }

        if (cheated == false)
        {
            if (progress != last_progress)
                UpdateReward();
            if (progress >= 0.99f)
                Goal();
        }
        
        // Small time penalty to encourage faster completion
        float reward = (float)aiController.Get("reward");
        aiController.Set("reward", reward -= 0.001f);

        last_progress = progress;
    }

    public void Cheat(Node3D body, float amount = 0.1f)    
    {
        float reward = (float)aiController.Get("reward");
        if (amount > 0.0f)
            aiController.Set("reward", reward - amount);
        cheated = true;
        Reset();
    }

    public void Goal()
	{
        var mesh = GetNode<MeshInstance3D>("MeshInstance3D") ;
        var mat = mesh.GetActiveMaterial(0) as StandardMaterial3D;
        mat.Emission = new Color(0,1,0);

        bestProgress = 9999.0f;
		float reward = (float)aiController.Get("reward");
        aiController.Set("reward", reward + 1.0f);
        GD.Print("Goal Reached " + (float)aiController.Get("reward"));
        Reset();
    }

    public void UpdateReward()
    {
        // Only reward forward progress
        float improvement = progress - last_progress;
        float best_imporvement = progress - bestProgress;

        // Apply reward only if moved forward
        float shapedReward = 0.0f;
        shapedReward = Mathf.Clamp(improvement * 10, -1f, 1f);
        // update max progress reached
        if (best_imporvement > 0.0f)
            bestProgress = progress;

        // Apply reward to agent
        float reward = (float)aiController.Get("reward");
        aiController.Set("reward", reward + shapedReward);
        GD.Print($"Reward applied {shapedReward}  Progress = {progress}  Best = {bestProgress}");
    }

    private void Reset()
    {
        Position = respawnMarker.Position;
        Velocity = Vector3.Zero;
        MoveAndSlide();
        timer = 0.0f;
        bestProgress = 0.0f;
        progress = 0.0f;
        aiController.Call("reset");
    }
}
