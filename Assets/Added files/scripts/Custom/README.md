# Custom UI System

This custom UI system allows you to create assignable buttons for saved command files with drag-and-drop functionality and program execution capabilities.

## Components

### 1. CreateUI.cs
The main script that manages the custom UI panel functionality.

**Features:**
- Opens/closes custom UI panel
- Displays dropdown of all saved command files
- Creates assignable buttons for each selected command file
- Manages Set toggle mode (drag vs assign mode)
- Integrates with existing SaveFile and Play systems

**Setup:**
1. Attach to a GameObject in your scene
2. Assign the following references:
   - `createUIButton`: Button that opens the custom UI panel
   - `customUIPanel`: The main custom UI panel GameObject
   - `commandFilesDropdown`: Dropdown showing saved command files
   - `setToggle`: Toggle for switching between drag and assign modes
   - `buttonContainer`: Transform where new buttons will be created
   - `buttonPrefab`: Prefab for the assignable buttons
   - `panelsToHide`: Array of panels to hide when custom UI is open
   - `saveFileScript`: Reference to SaveFile script
   - `playScript`: Reference to Play script
   - `uiManager`: Reference to UIManager script (optional)

### 2. DragHandler.cs
Handles drag-and-drop functionality for buttons.

**Features:**
- Implements Unity's drag interfaces (IBeginDragHandler, IDragHandler, IEndDragHandler)
- Visual feedback during dragging (semi-transparent)
- Drop zone detection
- Position restoration if drop fails

**Setup:**
1. Attach to button prefabs that should be draggable
2. Ensure the parent Canvas has a GraphicRaycaster component
3. The script will automatically find required components

### 3. DropZone.cs
Defines areas where draggable objects can be dropped.

**Features:**
- Configurable drop acceptance
- Position swapping or simple positioning
- Visual highlighting
- Customizable colors

**Setup:**
1. Attach to GameObjects that should act as drop zones
2. Configure drop settings in the inspector
3. Add Image component for visual feedback

### 4. UIManager.cs
Manages panel visibility and interactions.

**Features:**
- Automatically finds and manages UI panels
- Hides other panels when custom UI is open
- Panel tagging system for automatic discovery
- Centralized panel control

**Setup:**
1. Attach to a GameObject in your scene
2. Assign panels manually or use the tagging system
3. Reference from CreateUI script

## Usage

### Setting Up the UI

1. **Create the Custom UI Panel:**
   - Create a Canvas if you don't have one
   - Create a Panel GameObject for the custom UI
   - Add the necessary UI elements (dropdown, toggle, button container)

2. **Create the Button Prefab:**
   - Create a Button GameObject
   - Add DragHandler component
   - Add TMP_Text component for the button text
   - Save as a prefab

3. **Configure the Scripts:**
   - Attach CreateUI script to a GameObject
   - Assign all the required references in the inspector
   - Attach UIManager script and configure panel management

### Using the System

1. **Opening the Custom UI:**
   - Press the assigned "Create UI" button
   - The custom UI panel will open and hide all other panels
   - The dropdown will show all saved command files

2. **Creating Buttons:**
   - Select a command file from the dropdown
   - A button will be created with the file's name
   - The button will be added to the button container

3. **Set Toggle Modes:**

   **Set Toggle OFF (Drag Mode):**
   - Buttons are draggable
   - Can be repositioned by dragging
   - No program assignment functionality

   **Set Toggle ON (Assign Mode):**
   - Buttons are not draggable
   - Clicking a button will load and execute the assigned command file
   - Buttons are locked in position

4. **Executing Programs:**
   - With Set toggle ON, click any assigned button
   - The system will load the corresponding command file
   - The Play script will execute the loaded commands

## Integration with Existing Systems

### SaveFile Integration
- Uses existing SaveFile script to load command files
- Leverages the same file format and directory structure
- Added `LoadProgramByName()` method for external loading

### Play Script Integration
- Uses existing Play script for command execution
- Maintains compatibility with current robot control systems
- Supports all existing command types (MOVEJ, MOVEL, WAIT, SET)

### Panel Management
- Integrates with existing UI panels
- Automatically hides other panels when custom UI is active
- Maintains clean UI state management

## File Structure

```
Assets/Added files/scripts/Custom/
├── CreateUI.cs          # Main custom UI controller
├── DragHandler.cs       # Drag and drop functionality
├── DropZone.cs          # Drop zone management
├── UIManager.cs         # Panel management system
└── README.md           # This documentation
```

## Requirements

- Unity 2021.3 or later
- TextMeshPro package
- Existing SaveFile and Play scripts
- UI Canvas with GraphicRaycaster component

## Troubleshooting

### Buttons Not Dragging
- Ensure DragHandler component is attached to button prefab
- Check that parent Canvas has GraphicRaycaster component
- Verify that Set toggle is OFF

### Programs Not Executing
- Ensure SaveFile and Play scripts are properly referenced
- Check that command files exist in the correct directory
- Verify that Set toggle is ON when trying to execute

### Panels Not Hiding
- Check UIManager configuration
- Ensure panels are properly tagged or assigned
- Verify CreateUI script references

### Dropdown Not Populating
- Check that command files exist in the save directory
- Verify file format is correct (.json)
- Ensure SaveFile script is properly configured
