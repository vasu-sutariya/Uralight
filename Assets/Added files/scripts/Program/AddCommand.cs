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
		// ATTACH/DETACH
		public string attachedObjectName;

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
	[SerializeField] private Button attachButton;
	[SerializeField] private Button detachButton;

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
	[SerializeField] private GameObject attachPanel;
	[SerializeField] private GameObject detachPanel;

	[Header("Panel Delete Buttons")]
	[SerializeField] private Button moveJDeleteButton;
	[SerializeField] private Button moveLDeleteButton;
	[SerializeField] private Button waitDeleteButton;
	[SerializeField] private Button setDeleteButton;
	[SerializeField] private Button attachDeleteButton;
	[SerializeField] private Button detachDeleteButton;

	[Header("MOVEJ Action Buttons")]
	[SerializeField] private Button moveJSetButton; // Capture current joint angles
	[SerializeField] private Button moveJGotoButton; // Send MOVEJ command to robot

	[Header("MOVEL Action Buttons")]
	[SerializeField] private Button moveLSetButton; // Capture current joint angles
	[SerializeField] private Button moveLGotoButton; // Send MOVEJ command to robot

	[Header("MOVEJ Inputs")]
	[SerializeField] private TMP_InputField moveJBlendInput;
	[SerializeField] private TMP_InputField moveJTimeInput;
	[SerializeField] private TMP_InputField moveJSpeedInput;
	[SerializeField] private TMP_InputField moveJAccelerationInput;

	[Header("MOVEJ Toggles")]
	[SerializeField] private Toggle stopAtWaypointToggle;
	[SerializeField] private Toggle fixTimeToggle;
	[SerializeField] private Toggle maxAccAndVelToggle;

	[Header("MOVEJ Toggle GameObjects")]
	[SerializeField] private GameObject stopAtWaypointPanel1;      // First GameObject for blend radius parameter
	[SerializeField] private GameObject stopAtWaypointPanel2;      // Second GameObject for blend radius parameter
	[SerializeField] private GameObject fixTimePanel1;            // First GameObject for time parameter
	[SerializeField] private GameObject fixTimePanel2;            // Second GameObject for time parameter
	[SerializeField] private GameObject maxVelocityPanel1;        // First GameObject for velocity parameter
	[SerializeField] private GameObject maxVelocityPanel2;        // Second GameObject for velocity parameter
	[SerializeField] private GameObject maxAccelerationPanel1;    // First GameObject for acceleration parameter
	[SerializeField] private GameObject maxAccelerationPanel2;    // Second GameObject for acceleration parameter

	[Header("MOVEL Toggles")]
	[SerializeField] private Toggle stopAtWaypointToggleL;
	[SerializeField] private Toggle fixTimeToggleL;
	[SerializeField] private Toggle maxAccAndVelToggleL;

	[Header("MOVEL Toggle GameObjects")]
	[SerializeField] private GameObject stopAtWaypointPanelL1;      // First GameObject for blend radius parameter
	[SerializeField] private GameObject stopAtWaypointPanelL2;      // Second GameObject for blend radius parameter
	[SerializeField] private GameObject fixTimePanelL1;            // First GameObject for time parameter
	[SerializeField] private GameObject fixTimePanelL2;            // Second GameObject for time parameter
	[SerializeField] private GameObject maxVelocityPanelL1;        // First GameObject for velocity parameter
	[SerializeField] private GameObject maxVelocityPanelL2;        // Second GameObject for velocity parameter
	[SerializeField] private GameObject maxAccelerationPanelL1;    // First GameObject for acceleration parameter
	[SerializeField] private GameObject maxAccelerationPanelL2;    // Second GameObject for acceleration parameter

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

	[Header("ATTACH/DETACH Inputs")]
	[SerializeField] private TMP_Dropdown attachObjectDropdown;
	[SerializeField] private TMP_Dropdown detachObjectDropdown;

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
		WireUpToggles();
		HideAllPanels();
		PopulateObjectDropdowns();
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
		if (attachButton != null)
		{
			attachButton.onClick.RemoveAllListeners();
			attachButton.onClick.AddListener(() => AddCommandEntry("ATTACH"));
		}
		if (detachButton != null)
		{
			detachButton.onClick.RemoveAllListeners();
			detachButton.onClick.AddListener(() => AddCommandEntry("DETACH"));
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
		if (attachDeleteButton != null)
		{
			attachDeleteButton.onClick.RemoveAllListeners();
			attachDeleteButton.onClick.AddListener(DeleteSelectedCommand);
		}
		if (detachDeleteButton != null)
		{
			detachDeleteButton.onClick.RemoveAllListeners();
			detachDeleteButton.onClick.AddListener(DeleteSelectedCommand);
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

		// Wire up MOVEL action buttons
		if (moveLSetButton != null)
		{
			moveLSetButton.onClick.RemoveAllListeners();
			moveLSetButton.onClick.AddListener(SetMoveLTarget);
		}
		if (moveLGotoButton != null)
		{
			moveLGotoButton.onClick.RemoveAllListeners();
			moveLGotoButton.onClick.AddListener(GotoMoveLTarget);
		}
	}

	private void WireUpToggles()
	{
		// Wire up MOVEJ stop at waypoint toggle
		if (stopAtWaypointToggle != null)
		{
			stopAtWaypointToggle.onValueChanged.RemoveAllListeners();
			stopAtWaypointToggle.onValueChanged.AddListener(OnStopAtWaypointChanged);
		}

		// Wire up MOVEJ fix time toggle
		if (fixTimeToggle != null)
		{
			fixTimeToggle.onValueChanged.RemoveAllListeners();
			fixTimeToggle.onValueChanged.AddListener(OnFixTimeChanged);
		}

		// Wire up MOVEJ max acc and vel toggle
		if (maxAccAndVelToggle != null)
		{
			maxAccAndVelToggle.onValueChanged.RemoveAllListeners();
			maxAccAndVelToggle.onValueChanged.AddListener(OnMaxAccAndVelChanged);
		}

		// Wire up MOVEL stop at waypoint toggle
		if (stopAtWaypointToggleL != null)
		{
			stopAtWaypointToggleL.onValueChanged.RemoveAllListeners();
			stopAtWaypointToggleL.onValueChanged.AddListener(OnStopAtWaypointChangedL);
		}

		// Wire up MOVEL fix time toggle
		if (fixTimeToggleL != null)
		{
			fixTimeToggleL.onValueChanged.RemoveAllListeners();
			fixTimeToggleL.onValueChanged.AddListener(OnFixTimeChangedL);
		}

		// Wire up MOVEL max acc and vel toggle
		if (maxAccAndVelToggleL != null)
		{
			maxAccAndVelToggleL.onValueChanged.RemoveAllListeners();
			maxAccAndVelToggleL.onValueChanged.AddListener(OnMaxAccAndVelChangedL);
		}
	}

	private void OnStopAtWaypointChanged(bool isOn)
	{
		// When toggle is ON, disable blend panels. When OFF, enable blend panels
		if (stopAtWaypointPanel1 != null)
		{
			stopAtWaypointPanel1.SetActive(!isOn);
		}
		if (stopAtWaypointPanel2 != null)
		{
			stopAtWaypointPanel2.SetActive(!isOn);
		}
		
		// Set blend radius to 0 when toggle is ON (panels disabled)
		if (isOn && moveJBlendInput != null)
		{
			moveJBlendInput.text = "0";
		}
	}

	private void OnFixTimeChanged(bool isOn)
	{
		// If fix time is enabled, disable max acc and vel
		if (isOn && maxAccAndVelToggle != null)
		{
			maxAccAndVelToggle.isOn = false;
		}

		// Enable/disable entire fix time panel
		if (fixTimePanel1 != null)
		{
			fixTimePanel1.SetActive(isOn);
		}
		if (fixTimePanel2 != null)
		{
			fixTimePanel2.SetActive(isOn);
		}
		
		// Set time to 0 when disabled
		if (!isOn && moveJTimeInput != null)
		{
			moveJTimeInput.text = "0";
		}
	}

	private void OnMaxAccAndVelChanged(bool isOn)
	{
		// If max acc and vel is enabled, disable fix time
		if (isOn && fixTimeToggle != null)
		{
			fixTimeToggle.isOn = false;
		}

		// Enable/disable entire max acc and vel panel
		if (maxVelocityPanel1 != null)
		{
			maxVelocityPanel1.SetActive(isOn);
		}
		if (maxVelocityPanel2 != null)
		{
			maxVelocityPanel2.SetActive(isOn);
		}
		if (maxAccelerationPanel1 != null)
		{
			maxAccelerationPanel1.SetActive(isOn);
		}
		if (maxAccelerationPanel2 != null)
		{
			maxAccelerationPanel2.SetActive(isOn);
		}
		
		if (isOn)
		{
			// Set default values when enabled
			if (moveJSpeedInput != null)
			{
				moveJSpeedInput.text = "90";
			}
			if (moveJAccelerationInput != null)
			{
				moveJAccelerationInput.text = "90";
			}
		}
		else
		{
			// Set velocity and acceleration to 0 when disabled
			if (moveJSpeedInput != null)
			{
				moveJSpeedInput.text = "0";
			}
			if (moveJAccelerationInput != null)
			{
				moveJAccelerationInput.text = "0";
			}
		}
	}

	private void OnStopAtWaypointChangedL(bool isOn)
	{
		// When toggle is ON, disable blend panels. When OFF, enable blend panels
		if (stopAtWaypointPanelL1 != null)
		{
			stopAtWaypointPanelL1.SetActive(!isOn);
		}
		if (stopAtWaypointPanelL2 != null)
		{
			stopAtWaypointPanelL2.SetActive(!isOn);
		}
		
		// Set blend radius to 0 when toggle is ON (panels disabled)
		if (isOn && moveLBlendInput != null)
		{
			moveLBlendInput.text = "0";
		}
	}

	private void OnFixTimeChangedL(bool isOn)
	{
		// If fix time is enabled, disable max acc and vel
		if (isOn && maxAccAndVelToggleL != null)
		{
			maxAccAndVelToggleL.isOn = false;
		}

		// Enable/disable entire fix time panel
		if (fixTimePanelL1 != null)
		{
			fixTimePanelL1.SetActive(isOn);
		}
		if (fixTimePanelL2 != null)
		{
			fixTimePanelL2.SetActive(isOn);
		}
		
		// Set time to 0 when disabled
		if (!isOn && moveLTimeInput != null)
		{
			moveLTimeInput.text = "0";
		}
	}

	private void OnMaxAccAndVelChangedL(bool isOn)
	{
		// If max acc and vel is enabled, disable fix time
		if (isOn && fixTimeToggleL != null)
		{
			fixTimeToggleL.isOn = false;
		}

		// Enable/disable entire max acc and vel panel
		if (maxVelocityPanelL1 != null)
		{
			maxVelocityPanelL1.SetActive(isOn);
		}
		if (maxVelocityPanelL2 != null)
		{
			maxVelocityPanelL2.SetActive(isOn);
		}
		if (maxAccelerationPanelL1 != null)
		{
			maxAccelerationPanelL1.SetActive(isOn);
		}
		if (maxAccelerationPanelL2 != null)
		{
			maxAccelerationPanelL2.SetActive(isOn);
		}
		
		if (isOn)
		{
			// Set default values when enabled
			if (moveLSpeedInput != null)
			{
				moveLSpeedInput.text = "90";
			}
			if (moveLAccelerationInput != null)
			{
				moveLAccelerationInput.text = "90";
			}
		}
		else
		{
			// Set velocity and acceleration to 0 when disabled
			if (moveLSpeedInput != null)
			{
				moveLSpeedInput.text = "0";
			}
			if (moveLAccelerationInput != null)
			{
				moveLAccelerationInput.text = "0";
			}
		}
	}

	public void AddMoveJ() => AddCommandEntry("MOVEJ");
	public void AddMoveL() => AddCommandEntry("MOVEL");
	public void AddWait() => AddCommandEntry("WAIT");
	public void AddSet() => AddCommandEntry("SET");
	public void AddAttach() => AddCommandEntry("ATTACH");
	public void AddDetach() => AddCommandEntry("DETACH");

	public void ExecuteAttach(SimpleCommand cmd) => ExecuteAttachCommand(cmd);
	public void ExecuteDetach(SimpleCommand cmd) => ExecuteDetachCommand(cmd);
	
	/// <summary>
	/// Refreshes the object dropdowns - call this after importing new CAD models
	/// </summary>
	public void RefreshObjectDropdowns()
	{
		PopulateObjectDropdowns();
	}

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

	private void SetMoveLTarget()
	{
		if (selectedCommand == null || selectedCommand.commandType != "MOVEL") return;
		
		GameObject currentRobot = GetCurrentRobot();
		if (currentRobot == null) return;

		// Capture current pose
		selectedCommand.pose = GetCurrentPose(currentRobot);
		selectedCommand.isSet = true;

		// Update the command entry label to show it's set
		if (selectedEntry != null)
		{
			string robotName = selectedCommand.robotName;
			SetEntryLabel(selectedEntry, $"{robotName} - MOVEL [Set]");
		}
	}

	private void GotoMoveLTarget()
	{
		if (selectedCommand == null || selectedCommand.commandType != "MOVEL" || !selectedCommand.isSet) return;
		
		GameObject currentRobot = GetCurrentRobot();
		if (currentRobot == null) return;

		// Send MOVEL command to robot with blend radius and time
		MoveRobotToPose(currentRobot, selectedCommand.pose, selectedCommand.velocity, selectedCommand.acceleration, selectedCommand.blendRadius, selectedCommand.time);
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
			case "ATTACH":
				cmd.attachedObjectName = GetSelectedObjectName(attachObjectDropdown);
				cmd.isSet = true;
				break;
			case "DETACH":
				cmd.attachedObjectName = GetSelectedObjectName(detachObjectDropdown);
				cmd.isSet = true;
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
		
		// Refresh dropdowns in case new objects were added to the scene
		PopulateObjectDropdowns();
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
				
				// Set toggle states based on current values
				SetMoveJTogglesFromValues(cmd);
				break;
			case "MOVEL":
				if (moveLBlendInput != null) moveLBlendInput.SetTextWithoutNotify(cmd.blendRadius.ToString());
				if (moveLTimeInput != null) moveLTimeInput.SetTextWithoutNotify(cmd.time.ToString());
				if (moveLSpeedInput != null) moveLSpeedInput.SetTextWithoutNotify(cmd.velocity.ToString());
				if (moveLAccelerationInput != null) moveLAccelerationInput.SetTextWithoutNotify(cmd.acceleration.ToString());
				
				// Set toggle states based on current values
				SetMoveLTogglesFromValues(cmd);
				break;
			case "WAIT":
				if (waitTimeInput != null) waitTimeInput.SetTextWithoutNotify(cmd.time.ToString());
				break;
			case "SET":
				if (setIOPinInput != null) setIOPinInput.SetTextWithoutNotify(cmd.ioPin.ToString());
				if (setStatusDropdown != null) setStatusDropdown.SetValueWithoutNotify(cmd.status ? 1 : 0);
				break;
			case "ATTACH":
				SetDropdownSelection(attachObjectDropdown, cmd.attachedObjectName);
				break;
			case "DETACH":
				SetDropdownSelection(detachObjectDropdown, cmd.attachedObjectName);
				break;
		}
	}

	private void SetMoveJTogglesFromValues(SimpleCommand cmd)
	{
		if (cmd == null || cmd.commandType != "MOVEJ") return;

		// Set stop at waypoint toggle based on blend radius
		if (stopAtWaypointToggle != null)
		{
			bool shouldStopAtWaypoint = Mathf.Approximately(cmd.blendRadius, 0f);
			stopAtWaypointToggle.SetIsOnWithoutNotify(shouldStopAtWaypoint);
			
			// Update panel visibility based on toggle state
			if (stopAtWaypointPanel1 != null) stopAtWaypointPanel1.SetActive(!shouldStopAtWaypoint);
			if (stopAtWaypointPanel2 != null) stopAtWaypointPanel2.SetActive(!shouldStopAtWaypoint);
		}

		// Set fix time toggle based on time value
		if (fixTimeToggle != null)
		{
			bool shouldFixTime = !Mathf.Approximately(cmd.time, 0f);
			fixTimeToggle.SetIsOnWithoutNotify(shouldFixTime);
			
			// Update panel visibility based on toggle state
			if (fixTimePanel1 != null) fixTimePanel1.SetActive(shouldFixTime);
			if (fixTimePanel2 != null) fixTimePanel2.SetActive(shouldFixTime);
		}

		// Set max acc and vel toggle based on time and velocity values
		if (maxAccAndVelToggle != null)
		{
			bool shouldUseMaxAccAndVel = Mathf.Approximately(cmd.time, 0f) && 
										(!Mathf.Approximately(cmd.velocity, 0f) || !Mathf.Approximately(cmd.acceleration, 0f));
			maxAccAndVelToggle.SetIsOnWithoutNotify(shouldUseMaxAccAndVel);
			
			// Update panel visibility based on toggle state
			if (maxVelocityPanel1 != null) maxVelocityPanel1.SetActive(shouldUseMaxAccAndVel);
			if (maxVelocityPanel2 != null) maxVelocityPanel2.SetActive(shouldUseMaxAccAndVel);
			if (maxAccelerationPanel1 != null) maxAccelerationPanel1.SetActive(shouldUseMaxAccAndVel);
			if (maxAccelerationPanel2 != null) maxAccelerationPanel2.SetActive(shouldUseMaxAccAndVel);
		}
	}

	private void SetMoveLTogglesFromValues(SimpleCommand cmd)
	{
		if (cmd == null || cmd.commandType != "MOVEL") return;

		// Set stop at waypoint toggle based on blend radius
		if (stopAtWaypointToggleL != null)
		{
			bool shouldStopAtWaypoint = Mathf.Approximately(cmd.blendRadius, 0f);
			stopAtWaypointToggleL.SetIsOnWithoutNotify(shouldStopAtWaypoint);
			
			// Update panel visibility based on toggle state
			if (stopAtWaypointPanelL1 != null) stopAtWaypointPanelL1.SetActive(!shouldStopAtWaypoint);
			if (stopAtWaypointPanelL2 != null) stopAtWaypointPanelL2.SetActive(!shouldStopAtWaypoint);
		}

		// Set fix time toggle based on time value
		if (fixTimeToggleL != null)
		{
			bool shouldFixTime = !Mathf.Approximately(cmd.time, 0f);
			fixTimeToggleL.SetIsOnWithoutNotify(shouldFixTime);
			
			// Update panel visibility based on toggle state
			if (fixTimePanelL1 != null) fixTimePanelL1.SetActive(shouldFixTime);
			if (fixTimePanelL2 != null) fixTimePanelL2.SetActive(shouldFixTime);
		}

		// Set max acc and vel toggle based on time and velocity values
		if (maxAccAndVelToggleL != null)
		{
			bool shouldUseMaxAccAndVel = Mathf.Approximately(cmd.time, 0f) && 
										(!Mathf.Approximately(cmd.velocity, 0f) || !Mathf.Approximately(cmd.acceleration, 0f));
			maxAccAndVelToggleL.SetIsOnWithoutNotify(shouldUseMaxAccAndVel);
			
			// Update panel visibility based on toggle state
			if (maxVelocityPanelL1 != null) maxVelocityPanelL1.SetActive(shouldUseMaxAccAndVel);
			if (maxVelocityPanelL2 != null) maxVelocityPanelL2.SetActive(shouldUseMaxAccAndVel);
			if (maxAccelerationPanelL1 != null) maxAccelerationPanelL1.SetActive(shouldUseMaxAccAndVel);
			if (maxAccelerationPanelL2 != null) maxAccelerationPanelL2.SetActive(shouldUseMaxAccAndVel);
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
			case "ATTACH":
				if (attachObjectDropdown != null)
				{
					attachObjectDropdown.onValueChanged.AddListener(val => { cmd.attachedObjectName = GetSelectedObjectName(attachObjectDropdown); });
				}
				break;
			case "DETACH":
				if (detachObjectDropdown != null)
				{
					detachObjectDropdown.onValueChanged.AddListener(val => { cmd.attachedObjectName = GetSelectedObjectName(detachObjectDropdown); });
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

		if (attachObjectDropdown != null) attachObjectDropdown.onValueChanged.RemoveAllListeners();
		if (detachObjectDropdown != null) detachObjectDropdown.onValueChanged.RemoveAllListeners();
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
			case "ATTACH":
				if (attachPanel != null) attachPanel.SetActive(true);
				break;
			case "DETACH":
				if (detachPanel != null) detachPanel.SetActive(true);
				break;
		}
	}

	private void HideAllPanels()
	{
		if (moveJPanel != null) moveJPanel.SetActive(false);
		if (moveLPanel != null) moveLPanel.SetActive(false);
		if (waitPanel != null) waitPanel.SetActive(false);
		if (setPanel != null) setPanel.SetActive(false);
		if (attachPanel != null) attachPanel.SetActive(false);
		if (detachPanel != null) detachPanel.SetActive(false);
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

	private void MoveRobotToPose(GameObject robot, float[] targetPose, float velocity, float acceleration, float blendRadius = -1f, float time = -1f)
	{
		if (robot == null || targetPose == null || targetPose.Length < 6) return;
		UnityTrajControl traj = robot.GetComponentInChildren<UnityTrajControl>();
		if (traj != null)
		{
			traj.MoveL(targetPose, velocity, acceleration, blendRadius, time);
		}
	}

	private string GetSelectedObjectName(TMP_Dropdown dropdown)
	{
		if (dropdown == null || dropdown.options == null || dropdown.value < 0 || dropdown.value >= dropdown.options.Count)
			return string.Empty;
		
		return dropdown.options[dropdown.value].text;
	}

	private void SetDropdownSelection(TMP_Dropdown dropdown, string objectName)
	{
		if (dropdown == null || dropdown.options == null || string.IsNullOrEmpty(objectName))
			return;

		for (int i = 0; i < dropdown.options.Count; i++)
		{
			if (dropdown.options[i].text == objectName)
			{
				dropdown.SetValueWithoutNotify(i);
				return;
			}
		}
	}

	private bool IsRobot(GameObject go)
	{
		if (go == null) return false;
		if (go.GetComponentInChildren<UnityJointController>() != null) return true;
		if (go.GetComponentInChildren<ArticulationBody>() != null) return true;
		return false;
	}

	private bool IsCADImport(GameObject go)
	{
		if (go == null) return false;
		
		// Check if the object has a MeshFilter or MeshRenderer (typical for CAD imports)
		bool hasMeshFilter = go.GetComponent<MeshFilter>() != null;
		bool hasMeshRenderer = go.GetComponent<MeshRenderer>() != null;
		
		if (hasMeshFilter || hasMeshRenderer)
		{
			// Additional check: make sure it's not a UI element or other non-CAD object
			if (go.GetComponent<Canvas>() == null && go.GetComponent<CanvasRenderer>() == null)
			{
				return true;
			}
		}
		
		// Check if it's a child of a CAD import
		if (go.transform.parent != null && IsCADImport(go.transform.parent.gameObject)) return true;
		
		return false;
	}

	private void PopulateObjectDropdowns()
	{
		// Find all objects in the scene that can be attached/detached
		// Using the same logic as Jog_models.cs
		GameObject[] allObjects = FindObjectsByType<GameObject>(FindObjectsSortMode.None);
		List<string> objectNames = new List<string>();
		
		Debug.Log($"PopulateObjectDropdowns: Found {allObjects.Length} objects in scene");
		
		foreach (GameObject obj in allObjects)
		{
			if (obj == null) continue;
			
			// Check if it's a CAD import or robot (same logic as Jog_models)
			bool isRobot = IsRobot(obj);
			bool isCAD = IsCADImport(obj);
			
			if (isRobot || isCAD)
			{
				// Get the root object if it's a child
				GameObject rootObj = obj.transform.root.gameObject;
				
				// Only add if it's not already in the list
				if (!objectNames.Contains(rootObj.name))
				{
					objectNames.Add(rootObj.name);
					Debug.Log($"Added to dropdown: {rootObj.name} (Robot: {isRobot}, CAD: {isCAD})");
				}
			}
		}

		Debug.Log($"Total objects for dropdown: {objectNames.Count}");

		// Populate attach dropdown
		if (attachObjectDropdown != null)
		{
			attachObjectDropdown.ClearOptions();
			attachObjectDropdown.AddOptions(objectNames);
		}

		// Populate detach dropdown
		if (detachObjectDropdown != null)
		{
			detachObjectDropdown.ClearOptions();
			detachObjectDropdown.AddOptions(objectNames);
		}
	}

	private void ExecuteAttachCommand(SimpleCommand cmd)
	{
		if (cmd == null || string.IsNullOrEmpty(cmd.attachedObjectName)) return;

		GameObject currentRobot = GetCurrentRobot();
		if (currentRobot == null) return;

		// Find the object to attach
		GameObject objectToAttach = GameObject.Find(cmd.attachedObjectName);
		if (objectToAttach == null) return;

		// Find the attachment point (wrist3 for robots, or the robot itself for CAD imports)
		Transform targetParent = currentRobot.transform;
		if (IsRobot(currentRobot))
		{
			Transform wrist3 = FindRobotWrist3(currentRobot);
			if (wrist3 != null)
			{
				targetParent = wrist3;
			}
		}

		// Attach the object to the target parent
		objectToAttach.transform.SetParent(targetParent);
		
		Debug.Log($"Attached {objectToAttach.name} to {targetParent.name}");
	}

	private void ExecuteDetachCommand(SimpleCommand cmd)
	{
		if (cmd == null || string.IsNullOrEmpty(cmd.attachedObjectName)) return;

		// Find the object to detach
		GameObject objectToDetach = GameObject.Find(cmd.attachedObjectName);
		if (objectToDetach == null) return;

		// Check if the object has a parent
		if (objectToDetach.transform.parent != null)
		{
			// Store the current world position and rotation
			Vector3 worldPosition = objectToDetach.transform.position;
			Quaternion worldRotation = objectToDetach.transform.rotation;
			
			// Detach from parent
			objectToDetach.transform.SetParent(null);
			
			// Maintain the same world position and rotation
			objectToDetach.transform.position = worldPosition;
			objectToDetach.transform.rotation = worldRotation;
			
			Debug.Log($"Detached {objectToDetach.name} from parent");
		}
		else
		{
			Debug.Log($"{objectToDetach.name} is not attached to any parent");
		}
	}

	/// <summary>
	/// Finds the robot's wrist3 (last child) to use as the end effector attachment point
	/// </summary>
	private Transform FindRobotWrist3(GameObject robot)
	{
		if (robot == null) return null;

		// Get all children recursively
		Transform[] allChildren = robot.GetComponentsInChildren<Transform>();
		
		// Find the deepest child (highest depth)
		Transform wrist3 = null;
		int maxDepth = -1;
		
		foreach (Transform child in allChildren)
		{
			if (child == robot.transform) continue; // Skip the robot root
			
			int depth = GetTransformDepth(child, robot.transform);
			if (depth > maxDepth)
			{
				maxDepth = depth;
				wrist3 = child;
			}
		}
		
		return wrist3;
	}

	/// <summary>
	/// Calculates the depth of a transform relative to a root transform
	/// </summary>
	private int GetTransformDepth(Transform child, Transform root)
	{
		int depth = 0;
		Transform current = child;
		
		while (current != null && current != root)
		{
			depth++;
			current = current.parent;
		}
		
		return depth;
	}
}
