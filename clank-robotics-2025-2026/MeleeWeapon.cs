using Godot;
using System;
using System.Threading.Tasks;

public partial class MeleeWeapon : Powerable, Block
{
    [ExportGroup("MeleeWeapon Specific")]
    [Export] private float powerScale = 1.0f;
    [Export] public float Damage = 1f;
    [Export] public float Impact = 1f;
    [Export] public bool Powered = true;
    [Export] public bool Animated = true;
    private AnimationPlayer animationPlayer;
    private Area3D attackArea;
    private Timer attackTimer;

    public override void _Ready()
    {
        animationPlayer = GetNode<AnimationPlayer>("AnimationPlayer");
        attackArea = GetNode<Area3D>("AttackArea");
        attackTimer = GetNode<Timer>("AttackTimer");
        animationPlayer.Play("Animation");
        animationPlayer.SpeedScale = 0.0f;
        enginePower = 0.0f;
    }

    public override void SetPower(float input)
    {
        if (spawned)
        {
            input = Math.Clamp(input, -1f, 1f);
            enginePower = input * powerScale;
            attackTimer.WaitTime = Math.Abs(0.025f / (enginePower + 0.025f));
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        if (spawned)
        {
            animationPlayer.SpeedScale = enginePower;
            if (attackTimer.IsStopped())
                attackTimer.Start();
        }
    }

    public void AttackTimerTimeout()
    {
        if (Powered && spawned)
        {
            foreach (var body in attackArea.GetOverlappingBodies())
            {
                var target_body = body.GetParent();
                if (target_body is Block target && target_body.GetParent() != GetParent())
                {
                    target.ApplyDamage(Damage, GlobalPosition);
                }
            }
        }
        attackTimer.Start();
    }
}
