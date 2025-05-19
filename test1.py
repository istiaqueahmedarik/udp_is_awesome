import sys
import pygame
import time


def init_joystick():
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("No joystick detected. Connect one and try again.")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using joystick: {js.get_name()}")
    return js


def calibrate_dpad(js):
    """
    Press each D‑pad direction when prompted to record which button index it triggers.
    """
    print("\n=== DPAD CALIBRATION ===")
    mapping = {}
    for direction in ("up", "right", "down", "left"):
        input(
            f"  Press and hold the D‑pad {direction.upper()} then hit Enter…")
        pygame.event.pump()
        # scan all buttons for one currently pressed
        for b in range(js.get_numbuttons()):
            if js.get_button(b):
                mapping[direction] = b
                print(f"    → {direction} mapped to button {b}")
                break
        else:
            print("    ! No button detected—try again")
            return calibrate_dpad(js)
        time.sleep(0.2)  # debounce
    print("Calibration complete:", mapping)
    return mapping


def read_state(js, dpad_map):
    pygame.event.pump()
    # Axes
    axes = [js.get_axis(i) for i in range(js.get_numaxes())]
    # Buttons
    buttons = {i: js.get_button(i) for i in range(js.get_numbuttons())}
    # Hats (if any)
    hats = {i: js.get_hat(i) for i in range(js.get_numhats())}
    # Fallback dpad via button mapping
    dpad = {
        'up':    bool(buttons.get(dpad_map['up'], False)),
        'right': bool(buttons.get(dpad_map['right'], False)),
        'down':  bool(buttons.get(dpad_map['down'], False)),
        'left':  bool(buttons.get(dpad_map['left'], False)),
    }
    return axes, buttons, hats, dpad


def main():
    js = init_joystick()
    # If the controller already has hats, you can skip calibration:
    if js.get_numhats() == 0:
        dmap = calibrate_dpad(js)
    else:
        # default Pygame hat API will work:
        dmap = {}
        print("Controller already reports hats directly; skipping calibration.")
    print("\n=== Live input (Ctrl‑C to exit) ===")
    try:
        while True:
            axes, buttons, hats, dpad = read_state(js, dmap)
            # Display
            print("\nAxes:", ["{:.2f}".format(a) for a in axes])
            print("Buttons pressed:", [i for i, v in buttons.items() if v])
            if hats:
                print("Hats:", hats)
            print("D‑pad (mapped):", dpad)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting…")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
