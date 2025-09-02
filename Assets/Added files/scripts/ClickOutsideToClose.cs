using UnityEngine;
using UnityEngine.EventSystems;

public class ClickOutsideToClose : MonoBehaviour
{
    public GameObject panel;              // The panel to toggle
    public GameObject button;      // The button that shows the panel

    private bool isPanelOpen = false;

    void Update()
    {
        if (isPanelOpen && Input.GetMouseButtonDown(0)) // Left mouse click
        {
            // If the click is NOT over UI (e.g., clicked outside)
            if (!IsPointerOverUIObject())
            {
                HidePanel();
            }
            else if (!RectTransformUtility.RectangleContainsScreenPoint(
                         panel.GetComponent<RectTransform>(), Input.mousePosition, null) &&
                     !RectTransformUtility.RectangleContainsScreenPoint(
                         button.GetComponent<RectTransform>(), Input.mousePosition, null))
            {
                HidePanel();
            }
        }
    }

    public void TogglePanel()
    {
        isPanelOpen = !panel.activeSelf;
        panel.SetActive(isPanelOpen);
    }

    private void HidePanel()
    {
        panel.SetActive(false);
        isPanelOpen = false;
    }

    private bool IsPointerOverUIObject()
    {
        return EventSystem.current.IsPointerOverGameObject();
    }
}
