# Driver seat adjustment

Press F6 to open the driver-seat panel; F6, Escape or Close dismisses it.
Fore/aft, height and lateral sliders translate the shared cockpit eye anchor
in chassis coordinates. Positive fore/aft moves toward the steering wheel.
The default is 20 cm forward of the model's authored driver-eye socket.
Desktop and VR cockpit viewpoints both follow the adjustment immediately;
VR head tracking remains relative to the adjusted anchor.

Settings are saved per vehicle in user://seat_positions.cfg. Reset seat restores
the new default. Sliders support mouse, keyboard focus (Tab/arrows), and Godot's
standard UI navigation. The chassis and physical driving position do not change;
this is a seating/viewpoint adjustment, not deformation of the imported seat mesh.

While the panel is open, driving input is suppressed and the brake is held.
Camera look is disabled and the cursor is visible. Closing releases the brake
back to normal input; right mouse resumes mouse-look capture.
