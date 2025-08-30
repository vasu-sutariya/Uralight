using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class robotselect : MonoBehaviour
{
	[Header("Robot Selection Display")]
	[SerializeField] private Button robotDisplayButton; // Displays current robot name

	[Header("Providers")]
	[SerializeField] private Add robotManager; // source of current robot only

	void Awake()
	{
		if (robotManager == null)
		{
			robotManager = FindFirstObjectByType<Add>();
		}
	}

	void OnEnable()
	{
		UpdateRobotDisplayButton();
	}

	void Update()
	{
		UpdateRobotDisplayButton();
	}

	private void UpdateRobotDisplayButton()
	{
		if (robotDisplayButton == null) return;
		GameObject currentRobot = GetCurrentRobot();
		string robotName = currentRobot != null ? SanitizeName(currentRobot.name) : "No Robot";
		SetButtonText(robotDisplayButton, robotName);
	}

	private void SetButtonText(Button button, string text)
	{
		if (button == null) return;
		TMP_Text tmpText = button.GetComponentInChildren<TMP_Text>();
		if (tmpText != null)
		{
			tmpText.text = text;
			return;
		}
		Text uiText = button.GetComponentInChildren<Text>();
		if (uiText != null)
		{
			uiText.text = text;
		}
	}

	private GameObject GetCurrentRobot()
	{
		return robotManager != null ? robotManager.GetCurrentRobot() : null;
	}

	private static string SanitizeName(string raw)
	{
		if (string.IsNullOrEmpty(raw)) return string.Empty;
		return raw.Replace("(Clone)", string.Empty).Trim();
	}
}
