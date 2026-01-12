using Godot;
using System;
using System.Collections.Generic;

public partial class Robot : RigidBody3D
{

    public System.Collections.Generic.Dictionary<string, BaseNode> Nodes = new();
    public List<Placeable> Inputs = [];
    public List<Placeable> Outputs = [];
    public Vector3 PrevVelocity;

	public override void _Ready()
	{
		SignalBus signalBus = GetNode<SignalBus>("/root/SignalBus");
		signalBus.Connect(
			nameof(SignalBus.RobotSpawned),
			new Callable(this, nameof(SpawnRobot)));
	}

	private async void SpawnRobot(Node3D robot)
	{
		if (robot == this)
		{
			// Send spawn signal to all blocks
			foreach (Node3D child in GetChildren())
			{
				if (child is Block)
				{
					Block block = child as Block;
					if (block != null)
					{
						block.Spawn();
					}
				}
			}
            // Apply Gravity to robot
            await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
            GravityScale = 1.0f;
			// Create robot brain component
			var brain = GetNodeOrNull<RobotBrain>("RobotBrain") ?? new RobotBrain { Name = "RobotBrain" };
			if (brain.GetParent() != this) AddChild(brain);
			brain.Initialize(this);
			brain.ApplyLogicGraph(Nodes);
		}
	}

    // TODO
    private void UpdateMass()
    {
        Vector3 newCenterOfMass = new Vector3(0, 0, 0);
        float newMass = 0.0f;

        // Go through each block (child of type "Block") in robot
        // Get the Weight from them and sum up all the weight, then set current nodes "Mass" property to that
        // Correctly calculate center of mass based on offsets from current node and their Weight
        // Update current nodes "CenterOfMass" to be the new calculated center of mass
        // IN RobotRoot NODE UNDER MASS DISTRIBUTION CHANGE CENTER OF MASS TO CUSTOM!!!

        Mass = newMass;
        CenterOfMass = newCenterOfMass;
    }

    public async void LoadRobot(string robotName)
    {

        // Clear current robot
        foreach (var child in GetChildren())
        {
            child.QueueFree();
        }

        await ToSignal(GetTree(), "physics_frame"); // Wait 1 frame to ensure robot is cleared

        // Load new robot
        List<object> robotData = SaveLoad.LoadRobot(robotName);

        //Load Blocks
        List<Node3D> loadedBlockList = (List<Node3D>)robotData[0];
        foreach (Node3D block in loadedBlockList)
        {
            AddChild(block);
            MeshInstance3D meshInstance = block.GetNode<MeshInstance3D>("Mesh");
            UpdateBlockMaterial(block as Placeable, null);
        }

        //Load Script
        Nodes = (Dictionary<string, BaseNode>)robotData[1];

		//Updates Inputs and Outputs with new blocks
		foreach (Node3D child in GetChildren())
		{
			if (child is Placeable)
			{
				Placeable placeable = child as Placeable;
				if (placeable.IsInput) Inputs.Add((Placeable)child);
				if (placeable.IsOutput) Outputs.Add((Placeable)child);
			}
		}

		//Connect input blocks to input nodes and apply values
		foreach (BaseNode node in Nodes.Values)
		{
			switch (node.type)
			{
				case "CameraSensor":
					CameraSensorNode cn = node as CameraSensorNode;
					CameraSensor sensor = (CameraSensor)findInput(cn.SensorName);
					cn.CameraSensor = sensor;
					sensor.SensorRange = cn.Range;
					break;
				case "DistanceSensor":
					DistanceSensorNode dn = node as DistanceSensorNode;
					DistanceSensor sensor1 = (DistanceSensor)findInput(dn.SensorName);
					dn.DistanceSensor = sensor1;
					sensor1.SensorRange = dn.Range;
					break;
				default:
					break;
			}
		}
		//GD.Print("START-----------");
		//Connect output blocks to output nodes
		int i = 0;
		foreach (BaseNode node in Nodes.Values)
		{
			i++;
			//GD.Print(node.type);
			switch (node.type)
			{
				case "MeleeWeapon":
					GD.Print("ASSIGN HERE ");
					MeleeWeaponNode mn = node as MeleeWeaponNode;
					MeleeWeapon weapon = (MeleeWeapon)findOutput(mn.WeaponName);
					mn.Weapon = weapon;
					break;
				default:
					break;
			}
		}
		//GD.Print("END----------- " + i);

		// TESTING (EMIT SPAWN SIGNAL)
		//SignalBus signalBus = GetNode<SignalBus>("/root/SignalBus");
		//signalBus.EmitSignal(nameof(SignalBus.RobotSpawned), this);
	}

    private Sensor findInput(string name)
    {
        foreach (Sensor sensor in Inputs)
        {
            if (sensor.BlockName == name)
            {
                return sensor;
            }
        }
        GD.PrintErr("Found no input block with the name " + name);
        return null;
    }

	private Placeable findOutput(string name)
	{
		foreach (Placeable sensor in Outputs)
		{
			if (sensor.BlockName == name)
			{
				return sensor;
			}
		}
		GD.PrintErr("Found no output block with the name " + name);
		return null;
	}
	public void SaveRobot(string robotName)
	{
		SaveLoad.SaveRobot(this, robotName);
	}

    // Change from preview material to real and set color
    public void UpdateBlockMaterial(Placeable block, Object meshOverride)
    {
        MeshInstance3D meshInstance = block.GetNode<MeshInstance3D>("Mesh");
        if (meshOverride is MeshInstance3D)
        {
            meshInstance = meshOverride as MeshInstance3D;
        }
        SetMaterial(meshInstance, block as Placeable);
        foreach (MeshInstance3D childMesh in meshInstance.GetChildren())
        {
            SetMaterial(childMesh, block as Placeable);
        }
    }

	private void SetMaterial(MeshInstance3D meshInstance, Placeable block)
	{
		byte surfaceCount = (byte)meshInstance.Mesh.GetSurfaceCount();
		for (byte i = 0; i < surfaceCount; i++)
		{
			Material material = (Material)meshInstance.GetActiveMaterial(i);
			material.NextPass = null;
			Color currentColor = (Color)material.Get("albedo_color");
			currentColor.A = 1.0f;
			if (material != null && material.ResourceName == "Paintable")
			{
				currentColor = block.BlockColor;
				GD.Print(currentColor);
			}

            material.Set("albedo_color", currentColor);
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        PrevVelocity = LinearVelocity;
    }

    // Apply damage to local part damaged by collision
    private void Collision(Rid bodyRid, Node3D body, int bodyIndex, int localIndex)
    {
        var state = PhysicsServer3D.BodyGetDirectState(GetRid());

        // Local node
        var localNodeOwner = ShapeFindOwner(localIndex);
        var localNode = ShapeOwnerGetOwner(localNodeOwner);
        // Node of body collided with
        var bodyNodeOwner = ShapeFindOwner(bodyIndex);
        var bodyNode = ShapeOwnerGetOwner(bodyNodeOwner);

        for (int i = 0; i < state.GetContactCount(); i++)
        {
            if (state.GetContactCollider(i) == bodyRid)
            {
                Vector3 normal = state.GetContactLocalNormal(i);
                Vector3 relativeVelocity = PrevVelocity;
                if (body is Robot)
                {
                    Robot enemy = body as Robot;
                    relativeVelocity = (relativeVelocity - enemy.PrevVelocity);
                }
                float impactSpeed = (-relativeVelocity).Dot(normal);
                float damage = impactSpeed * 4f;

                if (impactSpeed >= 0.7f)
                {
                    DamageBlock((Node3D)localNode, damage, bodyNode as Node3D);
                }
            }
        }
    }
    public void DamageBlock(Node3D node, float damage, Node3D objectHit)
    {
        foreach (Node3D child in GetChildren())
        {
            if (child is Block)
            {
                Block block = child as Block;
                if (block != null)
                {
                    List<CollisionShape3D> blockColliders = block.GetColliders();
                    foreach (CollisionShape3D collider in blockColliders)
                    {
                        if (collider == node)
                        {
                            block.ApplyDamage(damage * 2.0f, objectHit.GlobalPosition);
                            break;
                        }
                    }
                }
            }
        }
    }
}
