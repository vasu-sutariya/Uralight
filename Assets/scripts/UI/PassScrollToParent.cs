using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class PassScrollToParent : MonoBehaviour, IScrollHandler
{
    public ScrollRect parentScroll;

    public void OnScroll(PointerEventData eventData)
    {
        if (parentScroll != null)
        {
            parentScroll.OnScroll(eventData);
        }
    }
}
