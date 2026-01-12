using Godot;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using static Godot.Light3D;


public partial class SaveLoad : Node
{
	// RobotName[BlockIndex][BlockStats...]
	//private static Dictionary<string, Dictionary<string, Dictionary<string, object>>> robotList = new Dictionary<string, Dictionary<string, Dictionary<string, object>>>();
	private static string robotListPath = "user://saves/robots/";

	public static void SaveRobot(Node3D robot, string robotName)
	{
		var saveData = new List<Dictionary<string, Dictionary<string, object>>>();
		saveData = Load(robotListPath) ?? new List<Dictionary<string, Dictionary<string, object>>>();

		//Create Dicts for Blocks and Script
		saveData.Add(new Dictionary<string, Dictionary<string, object>>());
		saveData.Add(new Dictionary<string, Dictionary<string, object>>());


		//Save blocks
		foreach (Node3D block in robot.GetChildren())
		{
			if (block is Placeable)
			{
				byte blockId = (byte)block.Get("BlockID");
				string blockName = (string)block.Get("BlockName");
				Vector3 position = block.Position;
				Vector3 rotation = block.Rotation;
				Color color = (Color)block.Get("BlockColor");
				//List<int> blockData = (List<int>)block.Get("blockData");
				string i = saveData[0].Count.ToString();
				saveData[0][i] = new Dictionary<string, object>(); // Creates new directory for each block
				saveData[0][i]["blockId"] = blockId;
				saveData[0][i]["blockname"] = blockName;
				saveData[0][i]["blockcolor"] = new int[] {(int)(color.R * 255), (int)(color.G * 255), (int)(color.B * 255) };
				saveData[0][i]["position"] = new float[] { position.X, position.Y, position.Z};
				saveData[0][i]["rotation"] = new float[] { rotation.X, rotation.Y, rotation.Z};
			}
		}

		//Save Script
		Robot r = robot as Robot;
		foreach (BaseNode node in r.Nodes.Values)
		{
			string i = saveData[1].Count.ToString();
			saveData[1][i] = new Dictionary<string, object>(); // Creates new directory for each script node
			saveData[1][i]["type"] = node.type;
			saveData[1][i]["internalname"] = node.InternalNodeName;
			saveData[1][i]["position"] = new float[] { node.position.X, node.position.Y };
			
			//Save references to connected nodes
			saveData[1][i]["references"] = new Dictionary<int, string>();
			int index = 0;
			foreach (BaseNode refNode in node.References.Values)
			{
				((Dictionary<int, string>)saveData[1][i]["references"])[index++] = refNode.InternalNodeName;
			}

			//Save which ports those references are connected to
			saveData[1][i]["referenceports"] = new Dictionary<int, int>();
			index = 0;
			foreach (int port in node.ReferencePorts.Values)
			{
				((Dictionary<int, int>)saveData[1][i]["referenceports"])[index++] = port;
			}

			//Special attributes for some nodes
			switch (node.type)
			{
				case "ConstantNumber":
					ConstantFloatNode fn = node as ConstantFloatNode;
					saveData[1][i]["amount"] = fn.Constant;
					break;
				case "ConstantBool":
					ConstantBoolNode bn = node as ConstantBoolNode;
					saveData[1][i]["state"] = bn.Constant;
					break;
				case "Timer":
					TimerNode tn = node as TimerNode;
					saveData[1][i]["loop"] = tn.Loop;
					saveData[1][i]["auto"] = tn.Auto;
					saveData[1][i]["interval"] = tn.Interval;
					break;
				case "Move":
					MoveNode mn = node as MoveNode;
					saveData[1][i]["speed"] = mn.Speed;
					break;
				case "Turn":
					TurnNode tn1 = node as TurnNode;
					saveData[1][i]["speed"] = tn1.Speed;
					break;
				case "CameraSensor":
					CameraSensorNode cn = node as CameraSensorNode;
					saveData[1][i]["selectedblockname"] = cn.SensorName;
					saveData[1][i]["range"] = cn.Range;
					break;
				case "DistanceSensor":
					DistanceSensorNode dn = node as DistanceSensorNode;
					saveData[1][i]["selectedblockname"] = dn.SensorName;
					saveData[1][i]["range"] = dn.Range;
					break;
				case "MeleeWeapon":
					MeleeWeaponNode mw = node as MeleeWeaponNode;
					saveData[1][i]["selectedblockname"] = mw.WeaponName;
					saveData[1][i]["power"] = mw.Power;
					break;
				default:
					break;
			}
		}
		Save(robotListPath + robotName + ".json", saveData);
	}


	public static List<object> LoadRobot(string robotName)
	{
		List<Dictionary<string, Dictionary<string, object>>> loadData = Load(robotListPath + robotName + ".json");
		List<Node3D> robotBlocks = new List<Node3D>();

		//Load block section
		foreach (var block in loadData[0])
		{
			var blockData = block.Value;
			var blockIdJson = (JsonElement)blockData["blockId"];
			var blockNameJson = (JsonElement)blockData["blockname"];
			var posArrayJson = (JsonElement)blockData["position"];
			var rotArrayJson = (JsonElement)blockData["rotation"];
			var colorArrayJson = (JsonElement)blockData["blockcolor"];

			byte blockId = blockIdJson.Deserialize<Byte>();             // block ID
			string blockName = blockNameJson.Deserialize<string>();
			float[] posArray = posArrayJson.Deserialize<float[]>();     // position
			float[] rotArray = rotArrayJson.Deserialize<float[]>();     // rotation
			int[] colorArray = colorArrayJson.Deserialize<int[]>();     // color

			PackedScene blockScene = BlockList.Blocks[blockId];
			Placeable instance = blockScene.Instantiate<Placeable>();
			instance.BlockName = blockName;
			instance.Position = new Vector3(posArray[0], posArray[1], posArray[2]);
			instance.Rotation = new Vector3(rotArray[0], rotArray[1], rotArray[2]);
			instance.BlockColor = new Color((float)colorArray[0] / 255f, (float)colorArray[1] / 255f, (float)colorArray[2] / 255f, 1);
			robotBlocks.Add(instance);
		}

		//Load script section

		//Save node references for connecting later
		Dictionary<string, Dictionary<int, string>> noderefs = [];

		Dictionary<string, BaseNode> nodes = [];

		foreach (var nodeData in loadData[1])
		{
			var nodeJson = nodeData.Value;
			var typeJson = (JsonElement)nodeJson["type"];
			var internalNameJson = (JsonElement)nodeJson["internalname"];
			var refsJson = (JsonElement)nodeJson["references"];
			var refportsJson = (JsonElement)nodeJson["referenceports"];
			var posJson = (JsonElement)nodeJson["position"];

			string type = typeJson.Deserialize<string>();
			string internalName = internalNameJson.Deserialize<string>();
			float[] fPos = posJson.Deserialize<float[]>();
			Vector2 position = new Vector2(fPos[0], fPos[1]);
			Godot.Collections.Dictionary<int, int> referenceports = refportsJson.Deserialize<Godot.Collections.Dictionary<int, int>>();

			//Save node references for connecting later
			Dictionary<int, string> references = refsJson.Deserialize<Dictionary<int, string>>();
			noderefs[internalName] = references;

			//Build nodes based on type and deserialize some type dependent values
			BaseNode node = null;
			switch (type)
			{
				case "Add":
					node = new AddNode();
					break;
				case "ConstantNumber":
					node = new ConstantFloatNode();
					var fn = node as ConstantFloatNode;
					var f = (JsonElement)nodeJson["amount"];
					fn.Constant = f.Deserialize<float>();
					break;
				case "ConstantBool":
					node = new ConstantBoolNode();
					var bn = node as ConstantBoolNode;
					var b = (JsonElement)nodeJson["state"];
					bn.Constant = b.Deserialize<bool>();
					break;
				case "And":
					node = new AndNode();
					break;
				case "Divide":
					node = new DivideNode();
					break;
				case "Equal":
					node = new EqualNode();
					break;
				case "GreaterThan":
					node = new GreaterThanNode();
					break;
				case "LessThan":
					node = new LessThanNode();
					break;
				case "Move":
					node = new MoveNode();
					var mn = node as MoveNode;
					var s = (JsonElement)nodeJson["speed"];
					mn.Speed = s.Deserialize<float>();
					break;
				case "Turn":
					node = new TurnNode();
					var tn = node as TurnNode;
					var s1 = (JsonElement)nodeJson["speed"];
					tn.Speed = s1.Deserialize<float>();
					break;
				case "Multiply":
					node = new MultiplyNode();
					break;
				case "Not":
					node = new NotNode();
					break;
				case "Or":
					node = new OrNode();
					break;
				case "Subtract":
					node = new SubtractNode();
					break;
				case "Timer":
					node = new TimerNode();
					var tn1 = node as TimerNode;
					var l = (JsonElement)nodeJson["loop"];
					var a = (JsonElement)nodeJson["auto"];
					var i = (JsonElement)nodeJson["interval"];
					tn1.Loop = l.Deserialize<bool>();
					tn1.Auto = a.Deserialize<bool>();
					tn1.Interval = i.Deserialize<float>();
					break;
				case "CameraSensor":
					node = new CameraSensorNode();
					var cn = node as CameraSensorNode;
					var sb = (JsonElement)nodeJson["selectedblockname"];
					var r = (JsonElement)nodeJson["range"];
					cn.SensorName = sb.Deserialize<string>();
					cn.Range = r.Deserialize<float>();
					break;
				case "DistanceSensor":
					node = new DistanceSensorNode();
					var dn = node as DistanceSensorNode;
					var sb1 = (JsonElement)nodeJson["selectedblockname"];
					var r1 = (JsonElement)nodeJson["range"];
					dn.SensorName = sb1.Deserialize<string>();
					dn.Range = r1.Deserialize<float>();
					break;
				case "MeleeWeapon":
					node = new MeleeWeaponNode();
					var mwn = node as MeleeWeaponNode;
					var sb2 = (JsonElement)nodeJson["selectedblockname"];
					var p = (JsonElement)nodeJson["power"];
					mwn.WeaponName = sb2.Deserialize<string>();
					mwn.Power = p.Deserialize<float>();
					break;
			}

			//Assign common values
			node.type = type;
			node.InternalNodeName = internalName;
			node.Name = internalName;
			node.ReferencePorts = referenceports;
			node.position = position;

			nodes[node.InternalNodeName] = node;
		}

		//Connect nodes with references
		//We have to convert from the saved internalNodeName to a reference to the node with that internalNodeName
		foreach (BaseNode node in nodes.Values)
		{
			Godot.Collections.Dictionary<int, BaseNode> realreferences = [];

			Dictionary<int, string> references = noderefs[node.InternalNodeName];

			//Go through every reference
			foreach (KeyValuePair<int, string> reference in references)
			{
				//and every node
				foreach(BaseNode refNode in nodes.Values)
				{
					//then find the node that has the same internalNodeName as the reference saved to disk
					if (refNode.InternalNodeName == reference.Value)
					{
						//and put that as the reference
						realreferences[reference.Key] = refNode;
					}
				}
			}

			node.References = realreferences;
		}

		//Cram everything into one 2D list
		List<Object> data = [];
		data.Add(robotBlocks);
		data.Add(nodes);

		return data;
	}

	public static List<string> GetRobotNames()
	{
		var dir = DirAccess.Open(robotListPath);
		if (dir == null)
		{
			GD.PrintErr("Save directory does not exist. No robots saved yet");
			return null;
		}
		List<string> nameList = new List<string> { };

		dir.ListDirBegin();
		string fileName = dir.GetNext();
		
		while (fileName != "")
		{
			if (!dir.CurrentIsDir())
			{
				if (!dir.CurrentIsDir())
				{
					nameList.Add(fileName.Replace(".json", "")); // Add robot name without .json extension to name list
				}
				fileName = dir.GetNext();
			}
		}

		return nameList;
	}


	// Helper functions
	// LOAD
	private static List<Dictionary<string, Dictionary<string, object>>> Load(string path)
	{
		if (Godot.FileAccess.FileExists(path) == false)
		{
			GD.PrintErr("File does not exist: " + path);
			return null;
		}
		// If file is valid
		using var file = Godot.FileAccess.Open(path, Godot.FileAccess.ModeFlags.Read);
		string jsonText = file.GetAsText();

		try
		{
			var loadData = JsonSerializer.Deserialize<List<Dictionary<string, Dictionary<string, object>>>>(jsonText);
			return loadData ?? new List<Dictionary<string, Dictionary<string, object>>>(); // Check formating and return
		}
		catch (JsonException e)
		{
			GD.PrintErr("Load error\nJSON Exception: " + e.Message);
			return null;
		}
	}

	// SAVE
	private static void Save(string path, List<Dictionary<string, Dictionary<string, object>>> content)
	{
		// Check that save folder exists, otherwise create it
		string dirPath = path.GetBaseDir();
		if (!DirAccess.DirExistsAbsolute(dirPath))
		{
			var result = DirAccess.MakeDirRecursiveAbsolute(dirPath);
			if (result != Godot.Error.Ok)
			{
				GD.PrintErr($"Failed to create directory: {dirPath} ({result})");
				return;
			}
		}


		using var file = Godot.FileAccess.Open(path, Godot.FileAccess.ModeFlags.Write);
		if (file == null)
		{
			GD.PrintErr("Failed to save file @ " + path);
			return;
		}

		// Save file
		string jsonString = JsonSerializer.Serialize(content, new JsonSerializerOptions
		{
			WriteIndented = true // Format for better readability
		});
		
		file.StoreString(jsonString);
	}
}
