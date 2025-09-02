using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI; // Add this for Button component
using System.Collections.Generic;
using System.Linq;

public class Jog_models : MonoBehaviour
{
	[Header("Selection Source")]
	[SerializeField] private Selctmodel selectionManager; // If null, will try FindObjectOfType

	[Header("UI References")]
	[SerializeField] private TMP_InputField[] positionInputs = new TMP_InputField[3]; // x,y,z (world position)
	[SerializeField] private TMP_InputField[] rotationInputs = new TMP_InputField[3]; // x,y,z (world euler angles in degrees)
	[SerializeField] private TMP_InputField[] scaleInputs = new TMP_InputField[3];    // x,y,z (local scale)
	[SerializeField] private Button deleteButton; // Add delete button reference
	[SerializeField] private Button detachButton; // Add detach button reference

	[Header("Parent Selection Dropdown")]
	[SerializeField] private TMP_Dropdown parentSelectionDropdown; // Dropdown for selecting parent object
	[SerializeField] private GameObject parentSelectionPanel; // Panel containing the dropdown (optional)

	[Header("Robot Placement")]
	[SerializeField] private Add addManager; // Provides MoveCurrentRobot

	private GameObject currentSelected;
	private bool transformInputsBeingEdited = false;
	private List<GameObject> availableParents = new List<GameObject>();

	void Awake()
	{
		if (selectionManager == null)
		{
			selectionManager = FindFirstObjectByType<Selctmodel>(FindObjectsInactive.Include);
		}
		if (addManager == null)
		{
			addManager = FindFirstObjectByType<Add>(FindObjectsInactive.Include);
		}
	}

	void Start()
	{
		// Wire up input listeners for position
		for (int i = 0; i < 3; i++)
		{
			int axis = i;
			if (positionInputs != null && i < positionInputs.Length && positionInputs[i] != null)
			{
				positionInputs[i].onSelect.AddListener(_ => OnAnyTransformInputSelected());
				positionInputs[i].onDeselect.AddListener(_ => OnAnyTransformInputDeselected());
				positionInputs[i].onEndEdit.AddListener(value => OnPositionEndEdit(axis, value));
			}
		}

		// Wire up input listeners for rotation
		for (int i = 0; i < 3; i++)
		{
			int axis = i;
			if (rotationInputs != null && i < rotationInputs.Length && rotationInputs[i] != null)
			{
				rotationInputs[i].onSelect.AddListener(_ => OnAnyTransformInputSelected());
				rotationInputs[i].onDeselect.AddListener(_ => OnAnyTransformInputDeselected());
				rotationInputs[i].onEndEdit.AddListener(value => OnRotationEndEdit(axis, value));
			}
		}

		// Wire up input listeners for scale
		for (int i = 0; i < 3; i++)
		{
			int axis = i;
			if (scaleInputs != null && i < scaleInputs.Length && scaleInputs[i] != null)
			{
				scaleInputs[i].onSelect.AddListener(_ => OnAnyTransformInputSelected());
				scaleInputs[i].onDeselect.AddListener(_ => OnAnyTransformInputDeselected());
				scaleInputs[i].onEndEdit.AddListener(value => OnScaleEndEdit(axis, value));
			}
		}

		// Wire up delete button
		if (deleteButton != null)
		{
			deleteButton.onClick.AddListener(OnDeleteButtonClicked);
		}

		// Wire up detach button
		if (detachButton != null)
		{
			detachButton.onClick.AddListener(OnDetachButtonClicked);
		}

		// Wire up parent selection dropdown
		if (parentSelectionDropdown != null)
		{
			parentSelectionDropdown.onValueChanged.AddListener(OnParentSelectionChanged);
		}
	}

	void Update()
	{
		// Track current selection from Selctmodel
		GameObject selected = selectionManager != null ? selectionManager.GetCurrentSelection() : null;
		if (selected != currentSelected)
		{
			currentSelected = selected;
			// Immediately reflect selection change
			UpdateDisplay();
			UpdateParentDropdown();
		}

		// While not editing, keep UI synced with current selection
		if (!transformInputsBeingEdited)
		{
			UpdateDisplay();
		}
	}

	private bool AnyInputFocused()
	{
		for (int i = 0; i < 3; i++)
		{
			if (positionInputs != null && i < positionInputs.Length && positionInputs[i] != null && positionInputs[i].isFocused) return true;
			if (rotationInputs != null && i < rotationInputs.Length && rotationInputs[i] != null && rotationInputs[i].isFocused) return true;
			if (scaleInputs != null && i < scaleInputs.Length && scaleInputs[i] != null && scaleInputs[i].isFocused) return true;
		}
		return false;
	}

	private void OnAnyTransformInputSelected()
	{
		transformInputsBeingEdited = true;
	}

	private void OnAnyTransformInputDeselected()
	{
		// Keep flag true until user finishes (set back to false after a successful apply in On...EndEdit)
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
		if (go.GetComponent<MeshFilter>() != null || go.GetComponent<MeshRenderer>() != null) return true;
		// Check if it's a child of a CAD import
		if (go.transform.parent != null && IsCADImport(go.transform.parent.gameObject)) return true;
		return false;
	}

	private void UpdateDisplay()
	{
		if (currentSelected == null)
		{
			ClearAllTransformInputs();
			SetDeleteButtonInteractable(false);
			SetDetachButtonInteractable(false);
			SetParentDropdownInteractable(false);
			return;
		}

		// Do not override while actively focused
		if (AnyInputFocused()) return;

		Transform t = currentSelected.transform;
		Vector3 pos = t.position;
		Vector3 rot = t.eulerAngles; // degrees
		Vector3 scl = t.localScale;

		SetInputTexts(positionInputs, pos);
		SetInputTexts(rotationInputs, rot);
		SetInputTexts(scaleInputs, scl);
		
		// Enable buttons when something is selected
		SetDeleteButtonInteractable(true);
		SetDetachButtonInteractable(true);
		SetParentDropdownInteractable(true);
	}

	private void UpdateParentDropdown()
	{
		if (parentSelectionDropdown == null) return;

		// Clear existing options
		parentSelectionDropdown.ClearOptions();
		availableParents.Clear();

		// Add a "None" option
		List<string> options = new List<string> { "None" };
		availableParents.Add(null);

		// Find all CAD imports and robots in the scene
		GameObject[] allObjects = FindObjectsByType<GameObject>(FindObjectsSortMode.None);
		
		foreach (GameObject obj in allObjects)
		{
			if (obj == null || obj == currentSelected) continue;

			// Check if it's a CAD import or robot
			if (IsRobot(obj) || IsCADImport(obj))
			{
				// Get the root object if it's a child
				GameObject rootObj = obj.transform.root.gameObject;
				
				// Only add if it's not already in the list and not the currently selected object
				if (!availableParents.Contains(rootObj) && rootObj != currentSelected)
				{
					availableParents.Add(rootObj);
					options.Add(rootObj.name);
				}
			}
		}

		// Set the dropdown options
		parentSelectionDropdown.AddOptions(options);
		
		// Set current selection to "None" by default
		parentSelectionDropdown.value = 0;
	}

	private void OnParentSelectionChanged(int dropdownIndex)
	{
		if (currentSelected == null || dropdownIndex <= 0) return;

		GameObject selectedParent = availableParents[dropdownIndex];
		if (selectedParent == null) return;

		// If the selected parent is a robot, find its wrist3 (last child) to act as end effector
		Transform targetParent = selectedParent.transform;
		if (IsRobot(selectedParent))
		{
			Transform wrist3 = FindRobotWrist3(selectedParent);
			if (wrist3 != null)
			{
				targetParent = wrist3;
			}
				
		}

		// Make the currently selected object a child of the target parent
		currentSelected.transform.SetParent(targetParent);
		
		// Reset dropdown to "None"
		parentSelectionDropdown.value = 0;
		
		Debug.Log($"Made {currentSelected.name} a child of {targetParent.name}");
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

	private void SetParentDropdownInteractable(bool interactable)
	{
		if (parentSelectionDropdown != null)
		{
			parentSelectionDropdown.interactable = interactable;
		}
		
		// Show/hide the parent selection panel if assigned
		if (parentSelectionPanel != null)
		{
			parentSelectionPanel.SetActive(interactable);
		}
	}

	private void SetInputTexts(TMP_InputField[] inputs, Vector3 values)
	{
		if (inputs == null || inputs.Length < 3) return;
		if (inputs[0] != null) inputs[0].text = values.x.ToString("F2");
		if (inputs[1] != null) inputs[1].text = values.y.ToString("F2");
		if (inputs[2] != null) inputs[2].text = values.z.ToString("F2");
	}

	private void SetDeleteButtonInteractable(bool interactable)
	{
		if (deleteButton != null)
		{
			deleteButton.interactable = interactable;
		}
	}

	private void SetDetachButtonInteractable(bool interactable)
	{
		if (detachButton != null)
		{
			detachButton.interactable = interactable;
		}
	}

	private void ClearAllTransformInputs()
	{
		ClearInputs(positionInputs);
		ClearInputs(rotationInputs);
		ClearInputs(scaleInputs);
	}

	private void ClearInputs(TMP_InputField[] inputs)
	{
		if (inputs == null) return;
		for (int i = 0; i < inputs.Length; i++)
		{
			if (inputs[i] != null) inputs[i].text = string.Empty;
		}
	}

	private void OnPositionEndEdit(int axis, string value)
	{
		if (currentSelected == null) return;
		if (!float.TryParse(value, out float v)) return;

		Transform t = currentSelected.transform;
		Vector3 pos = t.position;
		switch (axis)
		{
			case 0: pos.x = v; break;
			case 1: pos.y = v; break;
			case 2: pos.z = v; break;
		}

		if (IsRobot(currentSelected) && addManager != null)
		{
			addManager.MoveCurrentRobot(pos, t.rotation);
		}
		else
		{
			t.position = pos;
		}

		transformInputsBeingEdited = false;
	}

	private void OnRotationEndEdit(int axis, string value)
	{
		if (currentSelected == null) return;
		if (!float.TryParse(value, out float v)) return;

		Transform t = currentSelected.transform;
		Vector3 rot = t.eulerAngles; // degrees
		switch (axis)
		{
			case 0: rot.x = v; break;
			case 1: rot.y = v; break;
			case 2: rot.z = v; break;
		}

		if (IsRobot(currentSelected) && addManager != null)
		{
			Quaternion worldRot = Quaternion.Euler(rot);
			addManager.MoveCurrentRobot(t.position, worldRot);
		}
		else
		{
			t.eulerAngles = rot;
		}

		transformInputsBeingEdited = false;
	}

	private void OnScaleEndEdit(int axis, string value)
	{
		if (currentSelected == null) return;
		if (!float.TryParse(value, out float v)) return;

		Transform t = currentSelected.transform;
		Vector3 scl = t.localScale;
		switch (axis)
		{
			case 0: scl.x = v; break;
			case 1: scl.y = v; break;
			case 2: scl.z = v; break;
		}
		t.localScale = scl;

		transformInputsBeingEdited = false;
	}

	private void OnDeleteButtonClicked()
	{
		if (currentSelected == null) return;

		// Check if it's a robot and handle accordingly
		if (IsRobot(currentSelected) && addManager != null)
		{
			// Use addManager to remove robot if it has that functionality
			// You might need to implement RemoveCurrentRobot() in the Add class
			// For now, we'll just destroy the GameObject
		}

		// Destroy the selected GameObject
		DestroyImmediate(currentSelected);
		
		// Clear the selection
		currentSelected = null;
		
		// Update the display to clear inputs and disable delete button
		UpdateDisplay();
	}

	private void OnDetachButtonClicked()
	{
		if (currentSelected == null) return;

		// Check if the object has a parent
		if (currentSelected.transform.parent != null)
		{
			// Store the current world position and rotation
			Vector3 worldPosition = currentSelected.transform.position;
			Quaternion worldRotation = currentSelected.transform.rotation;
			
			// Detach from parent
			currentSelected.transform.SetParent(null);
			
			// Maintain the same world position and rotation
			currentSelected.transform.position = worldPosition;
			currentSelected.transform.rotation = worldRotation;
			
			Debug.Log($"Detached {currentSelected.name} from parent");
		}
		else
		{
			Debug.Log($"{currentSelected.name} is not attached to any parent");
		}
	}
} 