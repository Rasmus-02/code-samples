using Godot;
using System;
using System.Collections.Generic;
using static ParticleManager;

public interface Block
{
    void Spawn();
    void Delete();
    void ApplyDamage(float damage);
    void ApplyDamage(float damage, Vector3 damagePostion);
    void IsFloating();
    List<CollisionShape3D> GetColliders();
    List<Block> GetNeighbouringBlocks();
    bool IsCore();
    bool IsDestroyed();
}

public partial class Placeable : Node3D, Block
{
    [ExportGroup("Block Info")]
    [Export] public bool Core = false;
    [Export] public BlockList.BlockIDs BlockID = 0;
    [Export] public string BlockName = "";
    [Export] public bool IsInput = false;
    [Export] public bool IsOutput = false;
    [ExportGroup("Block Stats")]
    [Export] public float Health = 100f;
    [Export] public float Weight = 0.5f;
    [ExportGroup("Block Properties")]
    [Export] public bool Paintable = false;
    [Export] public bool Flamable = false;

    public Color BlockColor = Colors.White;
    public bool spawned = false;
    public List<CollisionShape3D> vehicleColliders = [];
    public RigidBody3D partRigidBody; // Used for parts that need a seperate rigid body for physics
    private Area3D snapArea;
    private List<Block> neighbouringBlocks = [];
    private bool destroyFlag;

    public override void _EnterTree()
    {
        SignalBus signalBus = GetNode<SignalBus>("/root/SignalBus");
        signalBus.Connect(
            nameof(SignalBus.FreeOrphanNodes),
            new Callable(this, nameof(FreeOrphan)));
    }

    public void ApplyDamage(float damage = 1.0f)
    {
        ApplyDamage(damage, GlobalPosition);
    }
    public void ApplyDamage(float damage, Vector3 damagePosition)
    {
        Health -= damage;

        // Damage Particle Effect
        Vector3 damageDir = GlobalPosition.DirectionTo(damagePosition);
        ParticleID particleID = ParticleManager.ParticleID.DAMAGE_PARTICLE;
        ParticleManager particleManager = GetTree().Root.GetNode<ParticleManager>("ParticleManager");
        particleManager.SpawnParticle(GlobalPosition, damageDir, particleID);
        
        if (Health <= 0f)
        {
            Delete();
        }
    }

    public virtual async void Spawn()
    {
        spawned = true;
        snapArea = (Area3D)GetNode("BuildingSnapArea");
        // Update hitbox
        StaticBody3D hitbox = GetNode<StaticBody3D>("Hitbox");
        for (int i = 0; i < hitbox.GetChildCount(); i++)
        {
            CollisionShape3D hitboxCollider = hitbox.GetChild<CollisionShape3D>(i);
            CollisionShape3D colliderInstance = (CollisionShape3D)hitboxCollider.Duplicate();
            vehicleColliders.Add(colliderInstance);
            colliderInstance.Owner = null;
            GetParent().AddChild(colliderInstance);
            colliderInstance.GlobalPosition = hitboxCollider.GlobalPosition;
            colliderInstance.GlobalRotation = hitboxCollider.GlobalRotation;
        }
        // Decrease Snap Area Size (prevents overlap)
        foreach (Node3D snapCollider in snapArea.GetChildren())
        {
            snapCollider.Scale = new Vector3(0.5f,0.5f,1.0f);
        }
        await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
        // Update neighbours
        foreach (Area3D neighbourSnapArea in snapArea.GetOverlappingAreas())
        {
            var neighbour = neighbourSnapArea.GetParent();
            if (neighbour is Block)
            {
                neighbouringBlocks.Add(neighbour as Block);
            }
        }
        // Delete floating blocks
        await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
        if (!IsCore())
            IsFloating();
    }

    public static void SpawnSpecific()
    {
    }

    public async void Delete()
    {
        destroyFlag = true;
        foreach (CollisionShape3D collider in vehicleColliders)
        {
            collider?.QueueFree();
        }
        // Recursively destroy floating blocks
        foreach (Block neighbour in neighbouringBlocks)
        {
            if (neighbour is Block && !neighbour.IsDestroyed())
                neighbour.IsFloating();
        }
        if (spawned)
        {
            Fracture();
        }
        await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
        partRigidBody?.QueueFree();
        vehicleColliders.Clear();
        QueueFree();
    }

    public void IsFloating()
    {
        if (!ConnectedToCore())
            Delete();
    }

    private bool ConnectedToCore()
    {
        var visited = new HashSet<Block>();
        var queue = new Queue<Block>();
        foreach (Block neighbour in neighbouringBlocks)
        {
            if (neighbour is Block && !neighbour.IsDestroyed())
            {
                queue.Enqueue(neighbour);
                visited.Add(neighbour);
            }
        }
        // Recursively go through all neighbours until connected neighbour tree has been searched
        while (queue.Count > 0)
        {
            Block current = queue.Dequeue();
            if (current.IsCore())
            {
                return true;
            }

            foreach (Block next in current.GetNeighbouringBlocks())
            {
                if (next is Block && !visited.Contains(next) && !next.IsDestroyed())
                {
                    visited.Add(next);
                    queue.Enqueue(next);
                }
            }
        }
        return false;
    }
    Node3D fracturedInstance;
    private void Fracture()
    {
        PackedScene fracturedScene = BlockList.BlocksFractured[(int)BlockID];
        fracturedInstance = fracturedScene.Instantiate<Node3D>();
        GetTree().CurrentScene.GetChild(1).AddChild(fracturedInstance);
        fracturedInstance.GlobalPosition = GlobalPosition;
        fracturedInstance.GlobalRotation = GlobalRotation;
        Robot robot = GetParent() as Robot;
        foreach (RigidBody3D child in fracturedInstance.GetChildren())
        {
            MeshInstance3D mesh = child.GetChild<MeshInstance3D>(0) as MeshInstance3D;
            robot.UpdateBlockMaterial(this, mesh);
        }
    }

    public List<CollisionShape3D> GetColliders()
    {
        return vehicleColliders;
    }
    public List<Block> GetNeighbouringBlocks()
    {
        return neighbouringBlocks;
    }
    public bool IsCore()
    {
        return Core;
    }
    public bool IsDestroyed()
    {
        return destroyFlag;
    }

    public void FreeOrphan()
    {
        if (GetParent() == null)
        {
            Delete();
        }
    }
}
