using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class Selctmodel : MonoBehaviour
{
	[Header("Selection Input")]
	[SerializeField] private Camera selectionCamera; // If null, will use Camera.main
	[SerializeField] private LayerMask selectableLayers = ~0; // Which layers can be selected
	[SerializeField] private bool deselectOnEmptyClick = true;
	[SerializeField] private bool deselectOnEscape = true;
	[SerializeField] private bool allowSelectionThroughUI = false; // Allow selection even when pointer is over UI

	[Header("Selection Targeting")]
	[SerializeField] private bool includeChildrenForHighlight = false; // Highlight only the selected object and its children

	[Header("Excluded Objects")]
	[SerializeField] public List<GameObject> excludedObjects = new List<GameObject>(); // Objects that should not be selectable

	[Header("Highlight (Optional)")]
	[SerializeField] private Material highlightMaterial; // Assign a simple highlight material (optional)
	[SerializeField] private bool keepOriginalMaterials = true; // Restore original materials on deselect

	[Header("Ray Visualization")]
	[SerializeField] private bool showRay = true;
	[SerializeField] private bool useLineRenderer = false; // If false, uses Debug.DrawRay
	[SerializeField] private Color rayColor = Color.green;
	[SerializeField] private float rayWidth = 0.02f;
	[SerializeField] private float rayDuration = 0.2f;
	[SerializeField] private float debugRayDistance = 100f; // Used when there's no hit

	[Header("Non-Collider Picking (Fallback)")]
	[SerializeField] private bool useBoundsPickingFallback = true; // Try Renderer.bounds ray intersection when no collider is hit
	[SerializeField] private bool onlyActiveObjectsForBoundsPick = true; // Skip inactive objects when bounds-picking

	[Header("UI Panel")]
	[SerializeField] private GameObject selectionPanel; // Panel to show/hide when object is selected

	[Header("Debug")]
	[SerializeField] private bool logSelection = false;

	private GameObject currentSelected;
	private readonly List<(Renderer renderer, Material[] original)> highlightedRenderers = new List<(Renderer, Material[])>();
	private LineRenderer rayLineRenderer;
	private float rayTimer = 0f;

	void Awake()
	{
		if (useLineRenderer && showRay)
		{
			SetupRayRenderer();
		}
	}

	void Update()
	{
		// Skip if pointer over UI
		if (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject())
		{
			//Debug.Log("Pointer over UI - selection blocked");
			if (!allowSelectionThroughUI)
				return;
		}

		// Escape to deselect
		if (deselectOnEscape && Input.GetKeyDown(KeyCode.Escape))
		{
			//Debug.Log("Escape");
			Deselect();
			return;
		}

		// Left click to select
		if (Input.GetMouseButtonDown(0))
		{
			//Debug.Log("Left click");
			HandleClickSelect(Input.mousePosition);
		}

		// Update ray line lifetime
		if (useLineRenderer && rayLineRenderer != null && rayTimer > 0f)
		{
			rayTimer -= Time.deltaTime;
			if (rayTimer <= 0f)
			{
				rayLineRenderer.enabled = false;
			}
		}
	}

	private void HandleClickSelect(Vector3 screenPosition)
	{
		//Debug.Log("HandleClickSelect");
		Camera cam = selectionCamera != null ? selectionCamera : Camera.main;
		if (cam == null)
		{
			//Debug.Log("No camera found");
			return;
		}

		Ray ray = cam.ScreenPointToRay(screenPosition);
		if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, selectableLayers))
		{
			//Debug.Log("Hit");
			bool isRobotLink = (hit.collider.gameObject.name.Contains("base") || 
							   hit.collider.gameObject.name.Contains("shoulder") ||
							   hit.collider.gameObject.name.Contains("wrist") ||
							   hit.collider.gameObject.name.Contains("arm")) && 
							   hit.collider.gameObject.GetComponentInParent<ArticulationBody>() != null;
			//Debug.Log($"Is Robot Link: {isRobotLink}");
			GameObject target = isRobotLink ? hit.collider.transform.root.gameObject : hit.collider.gameObject;
			
			// Check if the target is excluded from selection
			if (IsObjectExcluded(target))
			{
				//Debug.Log($"Object {target.name} is excluded from selection");
				VisualizeRay(ray, hit.point);
				return;
			}
			
			VisualizeRay(ray, hit.point);
			Select(target);
		}
		else
		{
			// Attempt bounds-based picking when no collider was hit
			if (useBoundsPickingFallback && TryBoundsRaycast(ray, out Renderer hitRenderer, out float boundsDistance))
			{
				bool isRobotLink = (hitRenderer.gameObject.name.Contains("base") || 
							   hitRenderer.gameObject.name.Contains("shoulder") ||
							   hitRenderer.gameObject.name.Contains("wrist") ||
							   hitRenderer.gameObject.name.Contains("arm") ||
							   hitRenderer.gameObject.name.Contains("elbow")) && 
							   hitRenderer.gameObject.GetComponentInParent<ArticulationBody>() != null;
				//Debug.Log($"Hit Renderer: {hitRenderer.gameObject.name}");
				//Debug.Log($"Is Robot Link: {isRobotLink}");
				GameObject target = isRobotLink ? hitRenderer.transform.root.gameObject : hitRenderer.gameObject;
				
				// Check if the target is excluded from selection			
				if (IsObjectExcluded(target))
				{
					//Debug.Log($"Object {target.name} is excluded from selection");
					VisualizeRay(ray, ray.origin + ray.direction * boundsDistance);
					return;
				}
				
				VisualizeRay(ray, ray.origin + ray.direction * boundsDistance);
				Select(target);
			}
			else if (deselectOnEmptyClick)
			{
				//Debug.Log("Deselecting");
				VisualizeRay(ray, ray.origin + ray.direction * debugRayDistance);
				Deselect();
			}
			else
			{
				//Debug.Log("No hit");
				VisualizeRay(ray, ray.origin + ray.direction * debugRayDistance);
			}
		}
	}

	public void Select(GameObject target)
	{
		if (target == currentSelected)
			return;

		Deselect();
		currentSelected = target;

		if (logSelection && currentSelected != null)
		{
			//Debug.Log($"Selected: {currentSelected.name}");
		}

		ApplyHighlight(currentSelected);

		// Enable selection panel
		if (selectionPanel != null)
		{
			selectionPanel.SetActive(true);
		}

		// NEW: Move/center camera orbit around the selected object
		if (currentSelected != null)
		{
			var cameraController = (selectionCamera != null ? selectionCamera : Camera.main)?.GetComponent<CameraController>();
			if (cameraController != null)
			{
				cameraController.SetPivotTo(currentSelected.transform);
			}
		}
	}

	public void Deselect()
	{
		if (currentSelected == null && highlightedRenderers.Count == 0)
			return;

		RestoreHighlight();
		if (logSelection && currentSelected != null)
		{
			//Debug.Log($"Deselected: {currentSelected.name}");
		}
		currentSelected = null;

		// Disable selection panel
		if (selectionPanel != null)
		{
			selectionPanel.SetActive(false);
		}
	}

	public GameObject GetCurrentSelection()
	{
		return currentSelected;
	}

	private void ApplyHighlight(GameObject target)
	{
		if (target == null)
			return;

		if (highlightMaterial == null)
			return; // No highlight requested

		// Get renderers based on the highlighting mode
		Renderer[] renderers;
		if (includeChildrenForHighlight)
		{
			// Highlight all descendants (original behavior)
			renderers = target.GetComponentsInChildren<Renderer>(true);
		}
		else
		{
			// Highlight only the selected object and its direct children
			renderers = GetSelectedObjectAndChildrenRenderers(target);
		}

		foreach (var r in renderers)
		{
			if (r == null)
				continue;

			Material[] original = null;
			if (keepOriginalMaterials)
			{
				// Clone the array to restore later
				var mats = r.sharedMaterials;
				original = new Material[mats.Length];
				for (int i = 0; i < mats.Length; i++) original[i] = mats[i];
			}

			// Build a temp array filled with the highlight material
			var highlightArray = new Material[r.sharedMaterials.Length];
			for (int i = 0; i < highlightArray.Length; i++) highlightArray[i] = highlightMaterial;
			r.sharedMaterials = highlightArray;

			if (keepOriginalMaterials)
			{
				highlightedRenderers.Add((r, original));
			}
		}
	}

	/// <summary>
	/// Gets renderers from the selected object and its direct children only
	/// </summary>
	private Renderer[] GetSelectedObjectAndChildrenRenderers(GameObject target)
	{
		List<Renderer> renderers = new List<Renderer>();
		
		// Add renderers from the target object itself
		Renderer[] targetRenderers = target.GetComponents<Renderer>();
		renderers.AddRange(targetRenderers);
		
		// Add renderers from direct children only (not grandchildren)
		for (int i = 0; i < target.transform.childCount; i++)
		{
			Transform child = target.transform.GetChild(i);
			Renderer[] childRenderers = child.GetComponents<Renderer>();
			renderers.AddRange(childRenderers);
		}
		
		return renderers.ToArray();
	}

	private void RestoreHighlight()
	{
		if (!keepOriginalMaterials)
		{
			// Nothing to restore
			highlightedRenderers.Clear();
			return;
		}

		for (int i = 0; i < highlightedRenderers.Count; i++)
		{
			var (renderer, original) = highlightedRenderers[i];
			if (renderer != null && original != null)
			{
				renderer.sharedMaterials = original;
			}
		}
		highlightedRenderers.Clear();
	}

	void OnDisable()
	{
		RestoreHighlight();
	}

	private void SetupRayRenderer()
	{
		rayLineRenderer = gameObject.GetComponent<LineRenderer>();
		if (rayLineRenderer == null)
			rayLineRenderer = gameObject.AddComponent<LineRenderer>();
		rayLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
		rayLineRenderer.startColor = rayColor;
		rayLineRenderer.endColor = rayColor;
		rayLineRenderer.startWidth = rayWidth;
		rayLineRenderer.endWidth = rayWidth;
		rayLineRenderer.positionCount = 2;
		rayLineRenderer.enabled = false;
	}

	private void VisualizeRay(Ray ray, Vector3 endPoint)
	{
		if (!showRay)
			return;

		if (useLineRenderer)
		{
			if (rayLineRenderer == null)
				SetupRayRenderer();
			rayLineRenderer.startColor = rayColor;
			rayLineRenderer.endColor = rayColor;
			rayLineRenderer.SetPosition(0, ray.origin);
			rayLineRenderer.SetPosition(1, endPoint);
			rayLineRenderer.enabled = true;
			rayTimer = rayDuration;
		}
		else
		{
			Debug.DrawRay(ray.origin, (endPoint - ray.origin), rayColor, rayDuration);
		}
	}

	private bool TryBoundsRaycast(Ray ray, out Renderer closestRenderer, out float closestDistance)
	{
		closestRenderer = null;
		closestDistance = float.MaxValue;

		Renderer[] allRenderers = FindObjectsByType<Renderer>(FindObjectsSortMode.None);
		for (int i = 0; i < allRenderers.Length; i++)
		{
			Renderer r = allRenderers[i];
			if (r == null)
				continue;

			GameObject go = r.gameObject;
			if (onlyActiveObjectsForBoundsPick && !go.activeInHierarchy)
				continue;

			if (!IsLayerInMask(go.layer, selectableLayers))
				continue;

			Bounds b = r.bounds;
			if (b.IntersectRay(ray, out float hitDistance))
			{
				if (hitDistance < closestDistance)
				{
					closestDistance = hitDistance;
					closestRenderer = r;
				}
			}
		}

		return closestRenderer != null;
	}

	private static bool IsLayerInMask(int layer, LayerMask mask)
	{
		return (mask.value & (1 << layer)) != 0;
	}

	private bool IsObjectExcluded(GameObject obj)
	{
		return excludedObjects.Contains(obj);
	}
}
