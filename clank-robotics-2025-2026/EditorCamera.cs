using Godot;
using System;


public partial class EditorCamera : SpringArm3D
{
	Editor editor = null;
	Camera3D camera = null;
	RayCast3D blockRaycast = null;
	float length = 0.0f;
	float yaw = 0.0f;
	float pitch = 0.0f;
	enum Modes
	{
		PLACE_BLOCK,
		PAINT_BLOCK,
    }
	Modes selectedMode = Modes.PLACE_BLOCK;


    public override void _Ready()
	{
		editor = (Editor)GetParent();
		camera = GetNode<Camera3D>("Camera");
		blockRaycast = GetNode<RayCast3D>("RayCast");
		length = SpringLength;
	}

	public override void _PhysicsProcess(double delta)
	{
		// Preview block placement
		Vector2 mousePosition = GetViewport().GetMousePosition();
		shoot_ray(mousePosition);
		
		if (blockRaycast.IsColliding() && selectedMode == Modes.PLACE_BLOCK)
		{
			Node3D collisionArea = (Node3D)blockRaycast.GetCollider();
			if (collisionArea is Area3D area)
			{
				byte colliderIndex = (byte)blockRaycast.GetColliderShape();
				Node3D collider = (Node3D)collisionArea.GetChild(colliderIndex);
				Marker3D placementPositionMarker = (Marker3D)collider.GetChild(0);
				Vector3 placementPosition = placementPositionMarker.GlobalPosition;
				editor.PreviewBlock(placementPosition, true);
			}
		}
		else
		{
			editor.PreviewBlock(Vector3.Zero, false);
		}

        // Switch editor mode (Place/Paint)
        // ========================================================= SHOULD IN THE FUTURE BE IN UI INSTEAD =========================================================
        if (Input.IsActionJustPressed("SwitchBuildMode"))
        {
            GD.Print("Current mode: ", selectedMode);
            if (selectedMode == Modes.PLACE_BLOCK)
                selectedMode = Modes.PAINT_BLOCK;
            else
                selectedMode = Modes.PLACE_BLOCK;
            GD.Print("Switched to mode: ", selectedMode);
        }
        //===========================================================================================================================================================
    }

    public override void _UnhandledInput(InputEvent @event)
	{
        // Rotate
        if (@event is InputEventMouseMotion mouseMotionEventRotation && Input.IsMouseButtonPressed(MouseButton.Middle))
		{
			Vector2 mouseDelta = mouseMotionEventRotation.Relative;

			yaw -= mouseDelta.X * 0.01f;
			pitch = Mathf.Clamp(pitch - mouseDelta.Y * 0.01f, -Mathf.Pi / 2, Mathf.Pi / 2);
			Rotation = new Vector3(pitch, yaw, 0); // Set rotation (no roll)
		}

		// Mouse Button events
		if (@event is InputEventMouseButton mouseButtonEvent)
		{
			// Zoom
			if (mouseButtonEvent.ButtonIndex == MouseButton.WheelUp && mouseButtonEvent.Pressed)
				length = Math.Max(length - 0.1f, 0.25f);
			else if (mouseButtonEvent.ButtonIndex == MouseButton.WheelDown && mouseButtonEvent.Pressed)
				length = Math.Min(length + 0.1f, 2.5f);
			SpringLength = length;

			// Block Handling
			Vector2 mousePosition = mouseButtonEvent.Position;
			shoot_ray(mousePosition);
			if (blockRaycast.IsColliding())
			{
				Node3D collisionArea = (Node3D)blockRaycast.GetCollider();
                if (mouseButtonEvent.Pressed)
				{
                    var block = collisionArea.GetParent() as Placeable;
                    
					// Place block
                    if (collisionArea is Area3D)
					{
                        byte colliderIndex = (byte)blockRaycast.GetColliderShape();
                        Node3D collider = (Node3D)collisionArea.GetChild(colliderIndex);
						Marker3D placementPositionMarker = (Marker3D)collider.GetChild(0);
						Vector3 placementPosition = placementPositionMarker.GlobalPosition;

						if (mouseButtonEvent.ButtonIndex == MouseButton.Left && collisionArea is Area3D area && selectedMode == Modes.PLACE_BLOCK)
						{
							GD.Print("Place ", block);
							editor.PlaceBlock(placementPosition);
						}
                    }
                    // Paint block
                    if (mouseButtonEvent.ButtonIndex == MouseButton.Left && selectedMode == Modes.PAINT_BLOCK)
					{
                        GD.Print("Remove ", block);
                        editor.PaintBlock(block as Placeable);
                    }
                    // Remove block
                    else if (mouseButtonEvent.ButtonIndex == MouseButton.Right && selectedMode == Modes.PLACE_BLOCK)
					{
                        GD.Print("Remove ", block);
                        editor.RemoveBlock(block);
                    }	
                }
			}
		}
    }

    public void shoot_ray(Vector2 mousePosition)
	{
		float rayLength = 100.0f;
		Vector3 globalTarget = camera.GlobalPosition + camera.ProjectRayNormal(mousePosition) * rayLength;
		blockRaycast.GlobalPosition = camera.GlobalPosition;
		blockRaycast.TargetPosition = blockRaycast.ToLocal(globalTarget);
    }

}
