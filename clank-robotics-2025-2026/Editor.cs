using Godot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using static SignalBus;

public partial class Editor : Node
{
	public Robot Robot = null;

    public BlockList.BlockIDs selectedBlockID { get; set; } = BlockList.BlockIDs.CUBE;
	private Vector3 selectedBlockRotation = new Vector3(0,0,0);
    private Color selectedColor = Colors.White;
    private PlacementChecker placementChecker = null;
	private MeshInstance3D blockPreview = null;
	private Color lastPreviewColor = new Color(1,1,1,1);
	private Area3D SnapArea = null;

	public override void _Ready()
	{
        Robot = GetNode<Robot>("RobotRoot");
		placementChecker = GetNode<PlacementChecker>("PlacementChecker");
    }

	public void StartWithRobot(string robotName)
	{
		LoadRobot(robotName);
        //Hacky way to get the robot name into UI
		LineEdit lineEdit = (LineEdit)FindChild("RobotNameField");
        lineEdit.Text = robotName;
    }

    public override void _UnhandledInput(InputEvent @event)
	{

		if (Input.IsActionJustPressed("RotateXLeft"))
		{
			selectedBlockRotation.X -= 90;
			AudioManager.Instance.PlaySFX("rotate_block");
		}
		if (Input.IsActionJustPressed("RotateXRight"))
		{
			selectedBlockRotation.X += 90;
			AudioManager.Instance.PlaySFX("rotate_block");
		}
		if (Input.IsActionJustPressed("RotateYRight"))
		{
			selectedBlockRotation.Y -= 90;
			AudioManager.Instance.PlaySFX("rotate_block");
		}
		if (Input.IsActionJustPressed("RotateYLeft"))
		{
			selectedBlockRotation.Y += 90;
			AudioManager.Instance.PlaySFX("rotate_block");
		}
		if (Input.IsActionJustPressed("RotateZForward"))
		{
			selectedBlockRotation.Z += 90;
			AudioManager.Instance.PlaySFX("rotate_block");
		}
		if (Input.IsActionJustPressed("RotateZBack"))
		{
			selectedBlockRotation.Z -= 90;
			AudioManager.Instance.PlaySFX("rotate_block");
		}

	}

	public async void PlaceBlock(Vector3 position)
	{
		// Check if block is placeable
		placementChecker.GlobalPosition = position;
		await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
		if (placementChecker.Check(SnapArea))
		{
			// Spawn block
			PackedScene block = BlockList.Blocks[(int)selectedBlockID];
			Placeable instance = (Placeable)block.Instantiate();
			Robot.AddChild(instance);
			instance.GlobalPosition = position;
			instance.RotationDegrees = selectedBlockRotation;

            // Set real material
			PaintBlock(instance);

            // Add place block to Inputs or Outputs depending on blocktype
            if (instance.IsInput) Robot.Inputs.Add(instance);
			if (instance.IsOutput) Robot.Outputs.Add(instance);

			// Give block name
			int safetyCounter = 0;
			int counter = 1;
			restartsearch:
			safetyCounter++;
			instance.BlockName = instance.BlockName.Remove(instance.BlockName.Length-1) + counter;
			foreach (Node node in Robot.GetChildren())
			{
				if (node is Placeable)
				{
					Placeable n = node as Placeable;
					if (n != instance && n.BlockName == instance.BlockName)
					{
						counter++;
						if (safetyCounter > 100) break;
						goto restartsearch;
					}
				}
			}

			AudioManager.Instance.PlaySFX("place_block");
		}
		PrintOrphanNodes();
        SignalBus signalBus = GetNode<SignalBus>("/root/SignalBus");
        signalBus.EmitSignal(nameof(SignalBus.FreeOrphanNodes));
    }

    public void SaveRobot(String name)
	{
		Robot.SaveRobot(name);
	}
	public void LoadRobot(String name)
	{
		Robot.LoadRobot(name);
	}

	public async void PreviewBlock(Vector3 position, bool showPreview)
	{
		placementChecker.GlobalPosition = position;
		placementChecker.RotationDegrees = selectedBlockRotation;

		// Delete block (refresh)
		if (IsInstanceValid(blockPreview))
		{
			blockPreview.QueueFree();
			blockPreview = null;
		}

		// Spawn Preview block
		if (showPreview)
		{
			PackedScene block = BlockList.Blocks[(int)selectedBlockID];
			Node3D blockInstance = (Node3D)block.Instantiate();
			blockPreview = (MeshInstance3D)blockInstance.GetNode<MeshInstance3D>("Mesh");
			blockInstance.RemoveChild(blockPreview);
			blockPreview.Owner = null;
			Robot.AddChild(blockPreview);
			blockPreview.GlobalPosition = position;
			blockPreview.RotationDegrees = selectedBlockRotation;
			byte surfaceCount = (byte)blockPreview.Mesh.GetSurfaceCount();
			SetPreviewColor(surfaceCount, lastPreviewColor); // Use last frames preview color (Needed because of async nature of function)
            blockInstance.QueueFree();

            // Remove old CollisionShapes and Areas from placement checker
            foreach (Node child in placementChecker.GetChildren())
			{
				child.QueueFree();
			}

			// Add new CollsinionShapes to placement checker (Prevent overlap)
			Node hitbox = blockInstance.GetNode("Hitbox");
			foreach (Node child in hitbox.GetChildren())
			{
				Node3D colliderClone = (Node3D)child.Duplicate();
				colliderClone.Owner = null;
				colliderClone.Scale = new Vector3(
					colliderClone.Scale.X * 0.95f,
					colliderClone.Scale.Y * 0.95f,
					colliderClone.Scale.Z * 0.95f);
				placementChecker.AddChild(colliderClone);
			}

			// Add new SnapArea to placement checker (Make sure only placeable on snappable surfaces)
			SnapArea = blockInstance.GetNode<Area3D>("BuildingSnapArea");
			SnapArea.Owner = null;
			blockInstance.RemoveChild(SnapArea);
			placementChecker.AddChild(SnapArea);
			SnapArea.CollisionLayer = 0;
			foreach (Node3D child in SnapArea.GetChildren())
			{
				child.Scale = new Vector3(
					child.Scale.X * 0.8f,
					child.Scale.Y * 0.8f,
					child.Scale.Z * 0.8f);
			}

			// Wait a frame for physics to update (othertwise placement check will be wrong)
			await ToSignal(GetTree(), SceneTree.SignalName.PhysicsFrame);
			if (placementChecker.Check(SnapArea))
				SetPreviewColor(surfaceCount, new Color(0, 0, 0, 0));
			else
				SetPreviewColor(surfaceCount, new Color(1, 0, 0, 1));
		}
	}

	public void RemoveBlock(Placeable block)
	{
		if (!block.Core)
			block.Delete();

		//Remove block from Inputs and Outputs
		if (Robot.Inputs.Contains(block)) Robot.Inputs.Remove(block);
		if (Robot.Outputs.Contains(block)) Robot.Outputs.Remove(block);
	}

	public void PaintBlock(Placeable block)
	{
		block.BlockColor = selectedColor;
        Robot.UpdateBlockMaterial(block, null);
    }

	private void SetPreviewColor(byte surfaceCount, Color color)
	{
		for (byte i = 0; i < surfaceCount; i++)
		{
			ShaderMaterial shaderMaterial = (ShaderMaterial)blockPreview.GetActiveMaterial(i).NextPass;
			shaderMaterial.SetShaderParameter("tint_color", color);
			shaderMaterial.SetShaderParameter("edge_color", color);
		}
		lastPreviewColor = color;
	}
}
