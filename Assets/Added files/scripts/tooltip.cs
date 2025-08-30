using UnityEngine;
using System.Collections;

public class TooltipController : MonoBehaviour
{
    public GameObject tooltip;     // Drag the tooltip GameObject in the Inspector
    public float delay = 0.5f;     // Time to wait before showing (in seconds)

    private Coroutine showCoroutine;

    // Call this on pointer enter (e.g., hover)
    public void ShowTooltipWithDelay()
    {
        // Start the coroutine to delay showing
        showCoroutine = StartCoroutine(ShowAfterDelay());
    }

    // Call this on pointer exit
    public void HideTooltip()
    {
        // Stop showing if user exits early
        if (showCoroutine != null)
            StopCoroutine(showCoroutine);

        tooltip.SetActive(false);
    }

    private IEnumerator ShowAfterDelay()
    {
        yield return new WaitForSeconds(delay);
        tooltip.SetActive(true);
    }
}
