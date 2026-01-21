using Godot;
using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text.Json;
using System.Threading.Tasks;

public partial class ApiService : Node
{
    private static string apiUrl = "http://localhost:8000/"; // Base URL for the API

    // Update or Add robot to server
    public static void SetRobot(string robotName, int weightclass, int matches_played, int matches_won, List<object> robotData)
	{
        GD.Print("Sending robot to server... " ,robotName);
        Dictionary<string, object> jsonHeader = CreateJsonHeader(robotName, weightclass, matches_played, matches_won, robotData);
        string json = (string)jsonHeader["json"];
        string[] headers = (string[])jsonHeader["headers"];
        string url = apiUrl + "robots";

        GD.Print("Sent clanker -> server");
        var httpRequest = new HttpRequest();
        Main.Instance.AddChild(httpRequest);
        httpRequest.RequestCompleted += (result, responseCode, respHeaders, body) =>
        {
            httpRequest.QueueFree();
        };
        httpRequest.Request(url, headers, Godot.HttpClient.Method.Post, json);
    }

    // Get robot by ID from server
    public static async Task<Dictionary<string, object>> GetRobot(string robotName)
    {
        string robotId = FormatUsername(robotName);
        string url = apiUrl + $"robots/{robotId}";

        var tcs = new TaskCompletionSource<Dictionary<string, object>>();

        var httpRequest = new HttpRequest();
        Main.Instance.AddChild(httpRequest);
        httpRequest.RequestCompleted += (result, responseCode, headers, body) =>
        {
            string json = System.Text.Encoding.UTF8.GetString(body);
            var data = JsonSerializer.Deserialize<Dictionary<string, object>>(json);
            tcs.SetResult(data);
            httpRequest.QueueFree();
        };

        GD.Print("Requested clanker <- server");
        httpRequest.Request(url);
        return await tcs.Task;
    }

    // Get list of opponents from server based on own robot stats
    public static async Task<List<List<object>>> GetOpponents(string robotName, int weightclass, int matches_played, int matches_won, List<object> robotData)
    {
        Dictionary<string, object> jsonHeader = CreateJsonHeader(robotName, weightclass, matches_played, matches_won, robotData);
        string json = (string)jsonHeader["json"];
        string[] headers = (string[])jsonHeader["headers"];
        string url = apiUrl + "matchmaking/opponents";

        var tcs = new TaskCompletionSource<List<List<object>>>();

        var httpRequest = new HttpRequest();
        Main.Instance.AddChild(httpRequest);
        httpRequest.RequestCompleted += (result, responseCode, responseHeaders, body) =>
        {
            string responseJson = System.Text.Encoding.UTF8.GetString(body);
            var serverRows = JsonSerializer.Deserialize<List<Dictionary<string, JsonElement>>>(responseJson);

            List<List<object>> formattedOpponents = new List<List<object>>();

            if (serverRows != null)
            {
                foreach (var row in serverRows)
                {
                    if (row.ContainsKey("robot_json"))
                    {
                        // 1. Convert the JsonElement into the specific Dictionary structure SaveLoad uses
                        var rawRobotData = row["robot_json"].Deserialize<List<Dictionary<string, Dictionary<string, object>>>>();

                        // Format robot data with SaveLoad Load function
                        var formattedOponent = SaveLoad.LoadRobot(row["robot_id"].ToString(), rawRobotData);

                        formattedOpponents.Add(formattedOponent);
                    }
                }
            }
            tcs.SetResult(formattedOpponents);
            httpRequest.QueueFree();
        };

        GD.Print("Requested opponents <- server");
        httpRequest.Request(url, headers, Godot.HttpClient.Method.Post, json);
        return await tcs.Task;
    }



    // Creates unique robot ID
    private static string FormatUsername(string robotName)
    {
        string user_id = Main.Instance.SteamID.ToString();
        string robotId = user_id + "_" + robotName;
        return robotId;
    }

    // Create Json header
    private static Dictionary<string, object> CreateJsonHeader(string robotName, int weightclass, int matches_played, int matches_won, List<object> robotData)
    {
        string user_id = Main.Instance.SteamID.ToString();
        string robotId = FormatUsername(robotName);


        // Needs to match RobotList Model in backend
        var robotDict = new Dictionary<string, object>
        {
            { "robot_id", robotId },
            { "user_id", user_id },
            { "matches_played", matches_played },
            { "matches_won", matches_won },
            { "weightclass", weightclass },
            { "robot_json", robotData }
        };

        string json = JsonSerializer.Serialize(robotDict);
        string[] headers = { "Content-Type: application/json" };

        var results = new Dictionary<string, object>
        {
            {"headers" , headers},
            {"json" , json},
        };

        return results;
    }
}
