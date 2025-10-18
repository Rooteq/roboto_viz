#!/usr/bin/env python3

import os
import random
import subprocess
import threading
from pathlib import Path
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot


class MusicPlayer(QObject):
    """Music player for robot navigation.

    Plays random MP3 files from ~/.robotroutes/music directory
    during navigation operations.
    """

    def __init__(self):
        super().__init__()
        self.music_dir = Path.home() / ".robotroutes" / "music"
        self.current_process = None
        self.is_playing = False
        self.should_continue_playing = False  # Flag to control continuous playback
        self._lock = threading.Lock()

        # Create music directory if it doesn't exist
        self.music_dir.mkdir(parents=True, exist_ok=True)

    def get_music_files(self):
        """Get list of MP3 files in the music directory."""
        if not self.music_dir.exists():
            return []

        mp3_files = list(self.music_dir.glob("*.mp3"))
        return mp3_files

    def start_music(self):
        """Start playing music continuously (enables continuous playback mode)."""
        # Set the flag to enable continuous playback
        self.should_continue_playing = True
        # Start the first track
        self._play_next_track()

    def _play_next_track(self):
        """Play a random music file from the music directory."""
        with self._lock:
            # If already playing, don't start another track
            if self.is_playing:
                print("DEBUG: Music already playing, not starting new track")
                return

            # Check if we should continue playing
            if not self.should_continue_playing:
                print("DEBUG: Continuous playback disabled, not starting new track")
                return

            # Get available music files
            music_files = self.get_music_files()

            if not music_files:
                print(f"WARNING: No MP3 files found in {self.music_dir}")
                return

            # Choose random file
            music_file = random.choice(music_files)

            try:
                # Use mpg123 or ffplay to play the music file
                # Try mpg123 first (lightweight), fallback to ffplay
                try:
                    self.current_process = subprocess.Popen(
                        ['mpg123', '-q', str(music_file)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    print(f"DEBUG: Started playing music: {music_file.name} using mpg123")
                except FileNotFoundError:
                    # mpg123 not available, try ffplay
                    try:
                        self.current_process = subprocess.Popen(
                            ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', str(music_file)],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL
                        )
                        print(f"DEBUG: Started playing music: {music_file.name} using ffplay")
                    except FileNotFoundError:
                        print("ERROR: Neither mpg123 nor ffplay found. Please install one to play music.")
                        print("  sudo apt-get install mpg123")
                        print("  or")
                        print("  sudo apt-get install ffmpeg")
                        return

                self.is_playing = True

                # Start a thread to monitor when the music finishes
                monitor_thread = threading.Thread(target=self._monitor_playback, daemon=True)
                monitor_thread.start()

            except Exception as e:
                print(f"ERROR: Failed to start music playback: {e}")
                self.is_playing = False
                self.current_process = None

    def stop_music(self):
        """Stop playing music and disable continuous playback."""
        # Disable continuous playback first
        self.should_continue_playing = False

        with self._lock:
            if not self.is_playing:
                return

            if self.current_process:
                try:
                    self.current_process.terminate()
                    self.current_process.wait(timeout=2.0)
                    print("DEBUG: Stopped music playback")
                except subprocess.TimeoutExpired:
                    # Force kill if terminate doesn't work
                    self.current_process.kill()
                    print("DEBUG: Force killed music playback")
                except Exception as e:
                    print(f"ERROR: Failed to stop music playback: {e}")
                finally:
                    self.current_process = None

            self.is_playing = False

    def _monitor_playback(self):
        """Monitor the playback process and start next track when it finishes."""
        if self.current_process:
            self.current_process.wait()
            with self._lock:
                if self.current_process and self.current_process.poll() is not None:
                    self.is_playing = False
                    self.current_process = None
                    print("DEBUG: Music playback finished naturally")

            # If continuous playback is enabled, play the next track
            if self.should_continue_playing:
                print("DEBUG: Starting next track...")
                self._play_next_track()

    @pyqtSlot(str)
    def handle_navigation_status(self, status: str):
        """Handle navigation status changes to start/stop music.

        Args:
            status: Navigation status string
        """
        # Define navigation states that should have music playing
        navigating_states = [
            "Nawigacja do celu",
            "Nawigacja do bazy",
            "Nav to dest",
            "Nav to base",
            "Navigating"
        ]

        # Define states where navigation has stopped
        stopped_states = [
            "Bezczynny",
            "Zatrzymany",
            "Na miejscu docelowym",
            "W bazie",
            "Failed",
            "Błąd Nawigacji",
            "Błąd Nav2",
            "Błąd - Cel nieosiągalny",
            "Błąd - Przekroczono czas nawigacji",
            "Stopped"
        ]

        # Check if we should be playing music
        if any(nav_state in status for nav_state in navigating_states):
            self.start_music()
        elif any(stop_state in status for stop_state in stopped_states):
            self.stop_music()

    def cleanup(self):
        """Cleanup resources when shutting down."""
        self.stop_music()
