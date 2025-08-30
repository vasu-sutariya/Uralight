using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class AddCommand : MonoBehaviour
{
	[System.Serializable]
	public class SimpleCommand
	{
		public string robotName;
		public string commandType;
		// MOVEJ
		public float[] angles = new float[6];
		// MOVEL
		public float[] pose = new float[6]; // [x,y,z, rx,ry,rz]
		// Shared params for MOVEJ/MOVEL, and time used by WAIT too
		public float blendRadius;
		public float time;
		public float velocity;
		public float acceleration;
		public bool isSet;
		// SET
		public int ioPin;
		public bool status;

		public SimpleCommand(string robotName, string commandType)
		{
			this.robotName = robotName;
			this.commandType = commandType;
		}
	}

	[Header("Command Buttons (Optional)")]
	[SerializeField] private Button moveJButton;
	[SerializeField] private Button moveLButton;
	[SerializeField] private Button waitButton;
	[SerializeField] private Button setButton;

	[Header("Delete Button")]
	[SerializeField] private Button deleteButton;

	[Header("UI")]
	[SerializeField] private RectTransform commandsList; // Parent with VerticalLayoutGroup
	[SerializeField] private GameObject commandEntryPrefab; // Prefab for a single program row (must contain a TMP_Text or Text)

	[Header("Parameter Panels")]
	[SerializeField] private GameObject moveJPanel;
	[SerializeField] private GameObject moveLPanel;
	[SerializeField] private GameObject waitPanel;
	[SerializeField] private GameObject setPanel;

	[Header("Panel Delete Buttons")]
	[SerializeField] private Button moveJDeleteButton;
	[SerializeField] private Button moveLDeleteButton;
	[SerializeField] private Button waitDeleteButton;
	[SerializeField] private Button setDeleteButton;

	[Header("MOVEJ Action Buttons")]
	[SerializeField] private Button moveJSetButton; // Capture current joint angles
	[SerializeField] private Button moveJGotoButton; // Send MOVEJ command to robot

	[Header("MOVEJ Inputs")]
	[SerializeField] private TMP_InputField moveJBlendInput;
	[SerializeField] private TMP_InputField moveJTimeInput;
	[SerializeField] private TMP_InputField moveJSpeedInput;
	[SerializeField] private TMP_InputField moveJAccelerationInput;

	[Header("MOVEL Inputs")]
	[SerializeField] private TMP_InputField moveLBlendInput;
	[SerializeField] private TMP_InputField moveLTimeInput;
	[SerializeField] private TMP_InputField moveLSpeedInput;
	[SerializeField] private TMP_InputField moveLAccelerationInput;

	[Header("WAIT Inputs")]
	[SerializeField] private TMP_InputField waitTimeInput;

	[Header("SET Inputs")]
	[SerializeField] private TMP_InputField setIOPinInput;
	[SerializeField] private TMP_Dropdown setStatusDropdown; // 0=OFF, 1=ON

	[Header("Providers")]
	[SerializeField] private Add robotManager; // source of current robot only

	// Simple in-memory list of added commands
	private readonly List<SimpleCommand> commands = new List<SimpleCommand>();
	private readonly List<GameObject> commandEntries = new List<GameObject>();
	private SimpleCommand selectedCommand = null;
	private GameObject selectedEntry = null;
	private Color selectedColor = new Color(0f, 0.784f, 1f, 1f); // #00C8FF
	private Color normalColor = Color.white;

	void Awake()
	{
		if (robotManager == null)
		{
			robotManager = FindFirstObjectByType<Add>();
		}
		WireUpButtons();
		HideAllPanels();
	}

	private void WireUpButtons()
	{
		if (moveJButton != null)
		{
			moveJButton.onClick.RemoveAllListeners();
			moveJButton.onClick.AddListener(() => AddCommandEntry("MOVEJ"));
		}
		if (moveLButton != null)
		{
			moveLButton.onClick.RemoveAllListeners();
			moveLButton.onClick.AddListener(() => AddCommandEntry("MOVEL"));
		}
		if (waitButton != null)
		{
			waitButton.onClick.RemoveAllListeners();
			waitButton.onClick.AddListener(() => AddCommandEntry("WAIT"));
		}
		if (setButton != null)
		{
			setButton.onClick.RemoveAllListeners();
			setButton.onClick.AddListener(() => AddCommandEntry("SET"));
		}
		if (deleteButton != null)
		{
			deleteButton.onClick.RemoveAllListeners();
			deleteButton.onClick.AddListener(DeleteSelectedCommand);
		}

		// Wire up panel delete buttons
		if (moveJDeleteButton != null)
		{
			moveJDeleteButton.onClick.RemoveAllListeners();
			moveJDeleteButton.onClick.AddListener(DeleteSelectedCommand);
		}
		if (moveLDeleteButton != null)
		{
			moveLDeleteButton.onClick.RemoveAllListeners();
			moveLDeleteButton.onClick.AddListener(DeleteSelectedCommand);
		}
		if (waitDeleteButton != null)
		{
			waitDeleteButton.onClick.RemoveAllListeners();
			waitDeleteButton.onClick.AddListener(DeleteSelectedCommand);
		}
		if (setDeleteButton != null)
		{
			setDeleteButton.onClick.RemoveAllListeners();
			setDeleteButton.onClick.AddListener(DeleteSelectedCommand);
		}

		// Wire up MOVEJ action buttons
		if (moveJSetButton != null)
		{
			moveJSetButton.onClick.RemoveAllListeners();
			moveJSetButton.onClick.AddListener(SetMoveJTarget);
		}
		if (moveJGotoButton != null)
		{
			moveJGotoButton.onClick.RemoveAllListeners();
			moveJGotoButton.onClick.AddListener(GotoMoveJTarget);
		}
	}

	public void AddMoveJ() => AddCommandEntry("MOVEJ");
	public void AddMoveL() => AddCommandEntry("MOVEL");
	public void AddWait() => AddCommandEntry("WAIT");
	public void AddSet() => AddCommandEntry("SET");

	public void DeleteSelectedCommand()
	{
		if (selectedCommand == null || selectedEntry == null) return;

		// Remove from data list
		int index = commands.IndexOf(selectedCommand);
		if (index >= 0)
		{
			commands.RemoveAt(index);
			commandEntries.RemoveAt(index);
		}

		// Destroy UI entry
		if (selectedEntry != null)
		{
			DestroyImmediate(selectedEntry);
		}

		// Clear selection and hide panels
		selectedCommand = null;
		selectedEntry = null;
		HideAllPanels();
		ClearPanelListeners();
	}

	private void SetMoveJTarget()
	{
		if (selectedCommand == null || selectedCommand.commandType != "MOVEJ") return;
		
		GameObject currentRobot = GetCurrentRobot();
		if (currentRobot == null) return;

		// Capture current joint angles
		selectedCommand.angles = GetCurrentJointAngles(currentRobot);
		selectedCommand.isSet = true;

		// Update the command entry label to show it's set
		if (selectedEntry != null)
		{
			string robotName = selectedCommand.robotName;
			SetEntryLabel(selectedEntry, $"{robotName} - MOVEJ [Set]");
		}
	}

	private void GotoMoveJTarget()
	{
		if (selectedCommand == null || selectedCommand.commandType != "MOVEJ" || !selectedCommand.isSet) return;
		
		GameObject currentRobot = GetCurrentRobot();
		if (currentRobot == null) return;

		// Send MOVEJ command to robot with blend radius and time
		MoveRobotToAngles(currentRobot, selectedCommand.angles, selectedCommand.velocity, selectedCommand.acceleration, selectedCommand.blendRadius, selectedCommand.time);
	}

	private void AddCommandEntry(string commandType)
	{
		// Determine robot name
		GameObject currentRobot = GetCurrentRobot();
		string robotName = currentRobot != null ? SanitizeName(currentRobot.name) : "No Robot";

		// Create command data and populate parameters based on type
		SimpleCommand cmd = new SimpleCommand(robotName, commandType);
		switch (commandType)
		{
			case "MOVEJ":
				cmd.blendRadius = ParseFloat(moveJBlendInput, 0.0f);
				cmd.time = ParseFloat(moveJTimeInput, 0.0f);
				cmd.velocity = ParseFloat(moveJSpeedInput, 90.0f);
				cmd.acceleration = ParseFloat(moveJAccelerationInput, 90.0f);
				cmd.angles = GetCurrentJointAngles(currentRobot);
				cmd.isSet = true;
				break;
			case "MOVEL":
				cmd.blendRadius = ParseFloat(moveLBlendInput, 0.0f);
				cmd.time = ParseFloat(moveLTimeInput, 0.0f);
				cmd.velocity = ParseFloat(moveLSpeedInput, 90.0f);
				cmd.acceleration = ParseFloat(moveLAccelerationInput, 90.0f);
				cmd.pose = GetCurrentPose(currentRobot);
				cmd.isSet = true;
				break;
			case "WAIT":
				cmd.time = ParseFloat(waitTimeInput, 1.0f);
				break;
			case "SET":
				cmd.ioPin = ParseInt(setIOPinInput, 0);
				cmd.status = GetStatus();
				break;
		}

		// Update UI if assigned
		if (commandsList != null && commandEntryPrefab != null)
		{
			string label = $"{robotName} - {commandType}";
			GameObject entry = Instantiate(commandEntryPrefab);
			entry.transform.SetParent(commandsList, false);
			(entry.transform as RectTransform)?.SetAsLastSibling();
			SetEntryLabel(entry, label);
			WireUpEntryButtons(entry, cmd);
			
			// Store references
			commandEntries.Add(entry);
		}

		// Record in simple data list
		commands.Add(cmd);
	}

	public int GetCommandCount() => commands.Count;
	public SimpleCommand GetCommandAt(int index) => (index >= 0 && index < commands.Count) ? commands[index] : null;
	public IReadOnlyList<SimpleCommand> GetCommands() => commands;
	
	// Methods for save/load functionality
	public void ClearAllCommands()
	{
		// Clear data
		commands.Clear();
		
		// Clear UI entries
		foreach (var entry in commandEntries)
		{
			if (entry != null)
			{
				DestroyImmediate(entry);
			}
		}
		commandEntries.Clear();
		
		// Clear selection
		selectedCommand = null;
		selectedEntry = null;
		
		// Hide panels
		HideAllPanels();
		ClearPanelListeners();
	}
	
	public void AddCommandProgrammatically(SimpleCommand command)
	{
		if (command == null) return;
		
		// Add to data list
		commands.Add(command);
		
		// Create UI entry if UI is available
		if (commandsList != null && commandEntryPrefab != null)
		{
			string label = $"{command.robotName} - {command.commandType}";
			if (command.isSet)
			{
				label += " [Set]";
			}
			
			GameObject entry = Instantiate(commandEntryPrefab);
			entry.transform.SetParent(commandsList, false);
			(entry.transform as RectTransform)?.SetAsLastSibling();
			SetEntryLabel(entry, label);
			WireUpEntryButtons(entry, command);
			
			// Store reference
			commandEntries.Add(entry);
		}
	}

	private void WireUpEntryButtons(GameObject entry, SimpleCommand cmd)
	{
		if (entry == null) return;
		Button[] buttons = entry.GetComponentsInChildren<Button>(true);
		if (buttons == null || buttons.Length == 0) return;
		foreach (var b in buttons)
		{
			b.onClick.AddListener(() => SelectCommand(cmd, entry));
		}
	}

	private void SelectCommand(SimpleCommand cmd, GameObject entry)
	{
		// Clear previous selection color
		if (selectedEntry != null)
		{
			SetEntryTextColor(selectedEntry, normalColor);
		}

		selectedCommand = cmd;
		selectedEntry = entry;
		
		// Set new selection color
		if (selectedEntry != null)
		{
			SetEntryTextColor(selectedEntry, selectedColor);
		}

		PopulatePanelFromCommand(cmd);
		ShowParamsPanelFor(cmd.commandType);
		AttachPanelListeners(cmd);
	}

	private void SetEntryTextColor(GameObject entry, Color color)
	{
		if (entry == null) return;
		TMP_Text tmp = entry.GetComponentInChildren<TMP_Text>(true);
		if (tmp != null)
		{
			tmp.color = color;
			return;
		}
		Text uiText = entry.GetComponentInChildren<Text>(true);
		if (uiText != null)
		{
			uiText.color = color;
		}
	}

	private void PopulatePanelFromCommand(SimpleCommand cmd)
	{
		if (cmd == null) return;
		switch (cmd.commandType)
		{
			case "MOVEJ":
				if (moveJBlendInput != null) moveJBlendInput.SetTextWithoutNotify(cmd.blendRadius.ToString());
				if (moveJTimeInput != null) moveJTimeInput.SetTextWithoutNotify(cmd.time.ToString());
				if (moveJSpeedInput != null) moveJSpeedInput.SetTextWithoutNotify(cmd.velocity.ToString());
				if (moveJAccelerationInput != null) moveJAccelerationInput.SetTextWithoutNotify(cmd.acceleration.ToString());
				break;
			case "MOVEL":
				if (moveLBlendInput != null) moveLBlendInput.SetTextWithoutNotify(cmd.blendRadius.ToString());
				if (moveLTimeInput != null) moveLTimeInput.SetTextWithoutNotify(cmd.time.ToString());
				if (moveLSpeedInput != null) moveLSpeedInput.SetTextWithoutNotify(cmd.velocity.ToString());
				if (moveLAccelerationInput != null) moveLAccelerationInput.SetTextWithoutNotify(cmd.acceleration.ToString());
				break;
			case "WAIT":
				if (waitTimeInput != null) waitTimeInput.SetTextWithoutNotify(cmd.time.ToString());
				break;
			case "SET":
				if (setIOPinInput != null) setIOPinInput.SetTextWithoutNotify(cmd.ioPin.ToString());
				if (setStatusDropdown != null) setStatusDropdown.SetValueWithoutNotify(cmd.status ? 1 : 0);
				break;
		}
	}

	private void AttachPanelListeners(SimpleCommand cmd)
	{
		ClearPanelListeners();
		if (cmd == null) return;
		switch (cmd.commandType)
		{
			case "MOVEJ":
				if (moveJBlendInput != null) moveJBlendInput.onValueChanged.AddListener(v => { cmd.blendRadius = ParseFloatImmediate(v, cmd.blendRadius); });
				if (moveJTimeInput != null) moveJTimeInput.onValueChanged.AddListener(v => { cmd.time = ParseFloatImmediate(v, cmd.time); });
				if (moveJSpeedInput != null) moveJSpeedInput.onValueChanged.AddListener(v => { cmd.velocity = ParseFloatImmediate(v, cmd.velocity); });
				if (moveJAccelerationInput != null) moveJAccelerationInput.onValueChanged.AddListener(v => { cmd.acceleration = ParseFloatImmediate(v, cmd.acceleration); });
				break;
			case "MOVEL":
				if (moveLBlendInput != null) moveLBlendInput.onValueChanged.AddListener(v => { cmd.blendRadius = ParseFloatImmediate(v, cmd.blendRadius); });
				if (moveLTimeInput != null) moveLTimeInput.onValueChanged.AddListener(v => { cmd.time = ParseFloatImmediate(v, cmd.time); });
				if (moveLSpeedInput != null) moveLSpeedInput.onValueChanged.AddListener(v => { cmd.velocity = ParseFloatImmediate(v, cmd.velocity); });
				if (moveLAccelerationInput != null) moveLAccelerationInput.onValueChanged.AddListener(v => { cmd.acceleration = ParseFloatImmediate(v, cmd.acceleration); });
				break;
			case "WAIT":
				if (waitTimeInput != null) waitTimeInput.onValueChanged.AddListener(v => { cmd.time = ParseFloatImmediate(v, cmd.time); });
				break;
			case "SET":
				if (setIOPinInput != null) setIOPinInput.onValueChanged.AddListener(v => { cmd.ioPin = ParseIntImmediate(v, cmd.ioPin); });
				if (setStatusDropdown != null)
				{
					setStatusDropdown.onValueChanged.AddListener(val => { cmd.status = (val != 0); });
				}
				break;
		}
	}

	private void ClearPanelListeners()
	{
		if (moveJBlendInput != null) moveJBlendInput.onValueChanged.RemoveAllListeners();
		if (moveJTimeInput != null) moveJTimeInput.onValueChanged.RemoveAllListeners();
		if (moveJSpeedInput != null) moveJSpeedInput.onValueChanged.RemoveAllListeners();
		if (moveJAccelerationInput != null) moveJAccelerationInput.onValueChanged.RemoveAllListeners();

		if (moveLBlendInput != null) moveLBlendInput.onValueChanged.RemoveAllListeners();
		if (moveLTimeInput != null) moveLTimeInput.onValueChanged.RemoveAllListeners();
		if (moveLSpeedInput != null) moveLSpeedInput.onValueChanged.RemoveAllListeners();
		if (moveLAccelerationInput != null) moveLAccelerationInput.onValueChanged.RemoveAllListeners();

		if (waitTimeInput != null) waitTimeInput.onValueChanged.RemoveAllListeners();

		if (setIOPinInput != null) setIOPinInput.onValueChanged.RemoveAllListeners();
		if (setStatusDropdown != null) setStatusDropdown.onValueChanged.RemoveAllListeners();
	}

	private void ShowParamsPanelFor(string commandType)
	{
		HideAllPanels();
		switch (commandType)
		{
			case "MOVEJ":
				if (moveJPanel != null) moveJPanel.SetActive(true);
				break;
			case "MOVEL":
				if (moveLPanel != null) moveLPanel.SetActive(true);
				break;
			case "WAIT":
				if (waitPanel != null) waitPanel.SetActive(true);
				break;
			case "SET":
				if (setPanel != null) setPanel.SetActive(true);
				break;
		}
	}

	private void HideAllPanels()
	{
		if (moveJPanel != null) moveJPanel.SetActive(false);
		if (moveLPanel != null) moveLPanel.SetActive(false);
		if (waitPanel != null) waitPanel.SetActive(false);
		if (setPanel != null) setPanel.SetActive(false);
	}

	private GameObject GetCurrentRobot()
	{
		return robotManager != null ? robotManager.GetCurrentRobot() : null;
	}

	private void SetEntryLabel(GameObject entry, string label)
	{
		if (entry == null) return;
		TMP_Text tmp = entry.GetComponentInChildren<TMP_Text>(true);
		if (tmp != null)
		{
			tmp.text = label;
			return;
		}
		Text uiText = entry.GetComponentInChildren<Text>(true);
		if (uiText != null)
		{
			uiText.text = label;
		}
	}

	private static string SanitizeName(string raw)
	{
		if (string.IsNullOrEmpty(raw)) return string.Empty;
		return raw.Replace("(Clone)", string.Empty).Trim();
	}

	private float ParseFloat(TMP_InputField input, float fallback)
	{
		if (input == null) return fallback;
		if (float.TryParse(input.text, out float v)) return v;
		return fallback;
	}

	private int ParseInt(TMP_InputField input, int fallback)
	{
		if (input == null) return fallback;
		if (int.TryParse(input.text, out int v)) return v;
		return fallback;
	}

	private float ParseFloatImmediate(string text, float fallback)
	{
		if (float.TryParse(text, out float v)) return v;
		return fallback;
	}

	private int ParseIntImmediate(string text, int fallback)
	{
		if (int.TryParse(text, out int v)) return v;
		return fallback;
	}

	private bool GetStatus()
	{
		if (setStatusDropdown == null) return false;
		return setStatusDropdown.value != 0;
	}

	private float[] GetCurrentJointAngles(GameObject robot)
	{
		if (robot == null) return new float[6];
		UnityEncoder encoder = robot.GetComponentInChildren<UnityEncoder>();
		if (encoder != null) return encoder.GetUnityAngles();
		return new float[6];
	}

	private float[] GetCurrentPose(GameObject robot)
	{
		float[] pose = new float[6];
		if (robot == null) return pose;

		UnityEncoder encoder = robot.GetComponentInChildren<UnityEncoder>();
		URInverseKinematics ik = robot.GetComponentInChildren<URInverseKinematics>();
		if (encoder != null && ik != null)
		{
			float[] angles = encoder.GetUnityAngles();
			var (pos, rot, _) = ik.GetEndEffectorPose(angles);
			pose[0] = pos.x; pose[1] = pos.y; pose[2] = pos.z;
			pose[3] = rot.x; pose[4] = rot.y; pose[5] = rot.z;
			return pose;
		}

		Transform endEffector = robot.transform.Find("EndEffector");
		if (endEffector != null)
		{
			Vector3 eul = endEffector.rotation.eulerAngles;
			pose[0] = endEffector.position.x; pose[1] = endEffector.position.y; pose[2] = endEffector.position.z;
			pose[3] = eul.x; pose[4] = eul.y; pose[5] = eul.z;
		}
		return pose;
	}

	private void MoveRobotToAngles(GameObject robot, float[] targetAngles, float velocity, float acceleration, float blendRadius = -1f, float time = -1f)
	{
		if (robot == null || targetAngles == null || targetAngles.Length < 6) return;
		UnityTrajControl traj = robot.GetComponentInChildren<UnityTrajControl>();
		if (traj != null)
		{
			traj.MoveJ(targetAngles, velocity, acceleration, blendRadius, time);
			return;
		}
		UnityJointController jointController = robot.GetComponentInChildren<UnityJointController>();
		if (jointController != null)
		{
			jointController.ChangeUnityTargetAngles(targetAngles);
		}
	}
}
