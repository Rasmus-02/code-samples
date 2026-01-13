using Godot;
using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text.Json;
using System.Threading.Tasks;

public partial class ApiService : Node
{
    private static string apiUrl = "http://localhost:8000/"; // Base URL for the API
    private static HttpRequest _httpRequest;


    public override void _Ready()
    {
        _httpRequest = new HttpRequest();
        AddChild(_httpRequest); // add it to the singleton
    }


    public static void SetRobot(string robotName, int matches_played, int matches_won, int weightclass)
	{
        string user_id = Main.Instance.SteamID.ToString();
        string robotId = FormatUsername(robotName);
        string url = apiUrl + "robots";


        var robotData = new Dictionary<string, object>
        {
            { "robot_id", robotId },
            { "user_id", user_id },
            { "matches_played", matches_played },
            { "matches_won", matches_won },
            { "weightclass", weightclass }
        };


        string json = JsonSerializer.Serialize(robotData);
        string[] headers = new string[] { "Content-Type: application/json" };

        GD.Print("Sent clanker -> server");
        _httpRequest.Request(url, headers, Godot.HttpClient.Method.Post, json);
    }

    public static async Task<Dictionary<string, object>> GetRobot(string robotName)
    {
        string robotId = FormatUsername(robotName);
        string url = apiUrl + $"robots/{robotId}";

        var tcs = new TaskCompletionSource<Dictionary<string, object>>();


        void OnRequestCompleted(long result, long responseCode, string[] headers, byte[] body)
        {
            string json = System.Text.Encoding.UTF8.GetString(body);
            var data = JsonSerializer.Deserialize<Dictionary<string, object>>(json);
            tcs.SetResult(data);

            _httpRequest.RequestCompleted -= OnRequestCompleted;
        }
        _httpRequest.RequestCompleted += OnRequestCompleted;

        GD.Print("Requested clanker <- server");
        _httpRequest.Request(url);
        return await tcs.Task;
    }


    // Creates unique robot ID
    private static string FormatUsername(string robotName)
    {
        string user_id = Main.Instance.SteamID.ToString();
        string robotId = user_id + "_" + robotName;
        return robotId;
    }
}
