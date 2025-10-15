# Changes Made to Launcher Application

## Version 2 - Fullscreen and Process Fix

### Changes Made:

#### 1. Fullscreen Mode (1920x1080)
- Changed from fixed 800x600 window to fullscreen mode
- Increased all font sizes for better visibility:
  - Title: 48pt (was 24pt)
  - Button text: 32pt (was 20pt)
  - Status text: 28pt (was 18pt)
  - Base font: 24pt (was 16pt)
- Increased button size: 150px height, 600px width (was 80px/400px)
- Changed background to dark theme (#2b2b2b) for better contrast
- White text on dark background for reduced eye strain

#### 2. Process Launching Fix
**Problem**: Processes were starting and immediately closing with "GUI closed" message.

**Root Cause**: QProcess.finished signal was being triggered immediately, likely because:
- Incorrect bash command formatting
- Signal connected before process fully started
- Missing proper exit code handling

**Solution**:
- Fixed bash command syntax: Removed extra quotes and corrected argument passing
- Changed from `"bash", ["-c", "bash -c 'command'"]` to `"/bin/bash", ["-c", "command"]`
- Added proper signal parameter handling: `on_gui_closed(exit_code, exit_status)`
- Added process state checking and proper error handling
- Implemented QTimer for non-blocking delay between process launches
- Added detailed debug output to track process lifecycle
- Added `waitForStarted()` checks to ensure processes actually launch

#### 3. Enhanced Debugging
- Added output redirection from both processes to console
- Print statements for tracking process state
- Error type detection and display
- Process exit codes logged

#### 4. Better Error Handling
- Proper exception handling in launch methods
- Process cleanup on errors
- User-friendly error messages in Polish
- Visual feedback with color-coded status

### Technical Details:

**Process Launch Sequence**:
1. User clicks "Uruchom tryb jazdy"
2. Diffbot process starts
3. Status shows "Uruchamianie diffbot..." (yellow)
4. Wait 2 seconds (non-blocking via QTimer)
5. GUI process starts
6. Status shows "System uruchomiony" (green)
7. Button disabled until GUI closes

**Process Termination**:
1. GUI closes (user clicks close button)
2. `on_gui_closed()` triggered with exit code
3. Diffbot process terminated gracefully
4. If doesn't terminate in 5 seconds, force kill
5. Button re-enabled
6. Status shows "Gotowy do uruchomienia" (white)

### Testing:

To test the changes:

```bash
cd /home/rooteq/ros2_ws/src/roboto_viz/launcher_app
python3 launcher.py
```

Expected behavior:
1. Fullscreen window appears
2. Click "Uruchom tryb jazdy"
3. Status changes to yellow "Uruchamianie diffbot..."
4. After 2 seconds, status changes to green "System uruchomiony"
5. Main GUI appears after ~10-20 seconds
6. Both processes run until close button clicked
7. Button becomes enabled again after closing

### Known Issues:

- Process output may be verbose in terminal (this is for debugging)
- Initial launch takes 10-20 seconds (normal for ROS2)
- If diffbot package doesn't exist, error will be shown

### Future Improvements:

- Add loading animation during startup
- Add process output log viewer
- Add individual process status indicators
- Add system health monitoring
- Add graceful shutdown timeout configuration
