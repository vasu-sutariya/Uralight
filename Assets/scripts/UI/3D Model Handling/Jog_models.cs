using System;
using TMPro;
using UnityEngine;

public class Jog_models : MonoBehaviour
{
	[Header("Selection Source")]
	[SerializeField] private Selctmodel selectionManager; // If null, will try FindObjectOfType

	[Header("UI References")]
	[SerializeField] private TMP_InputField[] positionInputs = new TMP_InputField[3]; // x,y,z (world position)
	[SerializeField] private TMP_InputField[] rotationInputs = new TMP_InputField[3]; // x,y,z (world euler angles in degrees)
	[SerializeField] private TMP_InputField[] scaleInputs = new TMP_InputField[3];    // x,y,z (local scale)

	[Header("Robot Placement")]
	[SerializeField] private Add addManager; // Provides MoveCurrentRobot

	private GameObject currentSelected;
	private bool transformInputsBeingEdited = false;

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

	private void UpdateDisplay()
	{
		if (currentSelected == null)
		{
			ClearAllTransformInputs();
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
	}

	private void SetInputTexts(TMP_InputField[] inputs, Vector3 values)
	{
		if (inputs == null || inputs.Length < 3) return;
		if (inputs[0] != null) inputs[0].text = values.x.ToString("F2");
		if (inputs[1] != null) inputs[1].text = values.y.ToString("F2");
		if (inputs[2] != null) inputs[2].text = values.z.ToString("F2");
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
} 