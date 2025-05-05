import subprocess
import signal
import sys
import argparse
import time


def setup_args():
    """Parse command-line arguments for the streaming configuration."""
    parser = argparse.ArgumentParser(
        description="Stream webcam using FFmpeg with GPU acceleration")
    parser.add_argument('-d', '--device', default='/dev/video0',
                        help='Video device (default: /dev/video0)')
    parser.add_argument('-r', '--resolution', default='1280x720',
                        help='Video resolution (default: 1280x720)')
    parser.add_argument('-f', '--fps', type=int, default=30,
                        help='Frames per second (default: 30)')
    parser.add_argument('-b', '--bitrate', default='2M',
                        help='Video bitrate (default: 2M)')
    parser.add_argument('-o', '--output', default='udp://127.0.0.1:23000',
                        help='Output destination (default: udp://127.0.0.1:23000)')
    parser.add_argument('-e', '--encoder', default='h264_nvenc',
                        choices=['h264_nvenc', 'h264_vaapi',
                                 'h264_qsv', 'libx264'],
                        help='Video encoder to use (default: h264_nvenc for NVIDIA GPUs)')
    parser.add_argument('-p', '--preset', default='fast',
                        help='Encoder preset (default: fast)')

    return parser.parse_args()


def start_streaming(args):
    """Start FFmpeg process for webcam streaming with GPU acceleration."""
    print(f"Starting webcam streaming with {args.encoder} encoder...")

    ffmpeg_cmd = [
        'ffmpeg',
        '-f', 'v4l2',                # Video4Linux2 input format
        '-framerate', str(args.fps),  # Input framerate
        '-video_size', args.resolution,  # Input resolution
        '-i', args.device,           # Input device
    ]

    # Add GPU acceleration options based on encoder
    if args.encoder == 'h264_nvenc':
        # NVIDIA GPU acceleration
        ffmpeg_cmd.extend([
            '-c:v', 'h264_nvenc',    # Use NVIDIA hardware encoder
            '-preset', args.preset,  # Encoding preset
            '-b:v', args.bitrate,    # Video bitrate
            '-profile:v', 'high',    # H.264 profile
        ])
    elif args.encoder == 'h264_vaapi':
        # VA-API (Intel, AMD) GPU acceleration
        ffmpeg_cmd.extend([
            '-vaapi_device', '/dev/dri/renderD128',  # VA-API device
            '-vf', f'format=nv12,hwupload',          # Upload to GPU memory
            '-c:v', 'h264_vaapi',                   # Use VA-API hardware encoder
            '-b:v', args.bitrate,                   # Video bitrate
        ])
    elif args.encoder == 'h264_qsv':
        # Intel QuickSync
        ffmpeg_cmd.extend([
            '-c:v', 'h264_qsv',      # Use QuickSync hardware encoder
            '-preset', args.preset,  # Encoding preset
            '-b:v', args.bitrate,    # Video bitrate
        ])
    else:
        # Software encoding with libx264
        ffmpeg_cmd.extend([
            '-c:v', 'libx264',       # Use x264 software encoder
            '-preset', args.preset,  # Encoding preset
            '-b:v', args.bitrate,    # Video bitrate
            '-tune', 'zerolatency',  # Tune for low latency
        ])

    # Common output options
    ffmpeg_cmd.extend([
        '-f', 'mpegts',              # Output format: MPEG Transport Stream
        args.output                  # Output destination
    ])

    print(f"Running command: {' '.join(ffmpeg_cmd)}")

    try:
        # Start FFmpeg process
        process = subprocess.Popen(
            ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        return process
    except Exception as e:
        print(f"Error starting FFmpeg: {e}")
        return None


def signal_handler(sig, frame):
    """Handle interrupt signals to clean up resources."""
    print("\nStopping streaming...")
    if 'process' in globals() and process:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    sys.exit(0)


if __name__ == "__main__":
    # Set up signal handlers for clean termination
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Parse command-line arguments
    args = setup_args()

    # Start streaming process
    process = start_streaming(args)

    if process:
        print("Streaming started. Press Ctrl+C to stop.")

        # Keep the script running
        try:
            while process.poll() is None:
                time.sleep(1)

            # If we get here, FFmpeg exited unexpectedly
            exit_code = process.returncode
            print(f"FFmpeg process exited unexpectedly with code {exit_code}")
        except KeyboardInterrupt:
            signal_handler(None, None)
    else:
        print("Failed to start streaming. Check your GPU and FFmpeg installation.")
