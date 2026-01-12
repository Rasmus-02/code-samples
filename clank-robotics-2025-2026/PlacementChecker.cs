using Godot;
using System;

// Used tp check if a block can be placed in the current position or if other block occupies the space
public partial class PlacementChecker : Area3D
{
	
	public bool Check(Area3D SnapArea)
	{
		bool canPlace = true;
		int blocksInsideCollider = GetOverlappingBodies().Count;
		if (blocksInsideCollider > 0)
			canPlace = false;
		
		int areasInsideSnapArea = SnapArea.GetOverlappingAreas().Count;
		if (areasInsideSnapArea <= 0)
			canPlace = false;
		
		return canPlace;
	}
}
