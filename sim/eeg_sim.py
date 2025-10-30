#!/usr/bin/env python3
"""
EEG Signal Simulator for SSVEP-based BCI
Generates synthetic 8-channel EEG data with embedded SSVEP responses
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os
from datetime import datetime

class EEGSimulator:
    def __init__(self, fs=256, duration=10, n_channels=8):
        """
        Initialize EEG Simulator
        
        Args:
            fs: Sampling frequency (Hz) - 256 Hz standard for EEG
            duration: Signal duration (seconds)
            n_channels: Number of EEG channels
        """
        self.fs = fs
        self.duration = duration
        self.n_channels = n_channels
        self.t = np.linspace(0, duration, fs * duration, endpoint=False)
        
        # EEG channel names (standard 10-20 system positions)
        self.channel_names = ['O1', 'O2', 'Oz', 'P3', 'P4', 'Pz', 'C3', 'C4'][:n_channels]
        
        # SSVEP stimulus frequencies for robot commands
        self.ssvep_freqs = {
            'left': 10.0,   # 10 Hz - rotate left
            'right': 12.0,  # 12 Hz - rotate right  
            'up': 15.0,     # 15 Hz - move up
            'down': 8.5     # 8.5 Hz - move down
        }
        
        print(f"EEG Simulator initialized: {n_channels} channels, {fs} Hz, {duration}s")
    
    def generate_base_rhythms(self):
        """Generate base EEG rhythms (alpha, beta, theta)"""
        eeg_data = np.zeros((self.n_channels, len(self.t)))
        
        for ch in range(self.n_channels):
            # Alpha rhythm (8-13 Hz) - dominant in occipital regions
            alpha_amp = 20 if 'O' in self.channel_names[ch] else 10
            alpha_freq = np.random.uniform(9, 11)  # Individual alpha frequency
            alpha = alpha_amp * np.sin(2 * np.pi * alpha_freq * self.t + np.random.rand() * 2 * np.pi)
            
            # Beta rhythm (13-30 Hz) - lower amplitude
            beta_amp = 5
            beta_freq = np.random.uniform(18, 25)
            beta = beta_amp * np.sin(2 * np.pi * beta_freq * self.t + np.random.rand() * 2 * np.pi)
            
            # Theta rhythm (4-8 Hz) - very low amplitude
            theta_amp = 3
            theta_freq = np.random.uniform(5, 7)
            theta = theta_amp * np.sin(2 * np.pi * theta_freq * self.t + np.random.rand() * 2 * np.pi)
            
            # Combine rhythms
            eeg_data[ch] = alpha + beta + theta
            
            # Add pink noise (1/f noise - common in biological signals)
            pink_noise = self.generate_pink_noise(len(self.t)) * 5
            eeg_data[ch] += pink_noise
            
            # Add white noise
            white_noise = np.random.randn(len(self.t)) * 2
            eeg_data[ch] += white_noise
        
        return eeg_data
    
    def generate_pink_noise(self, n_samples):
        """Generate pink (1/f) noise"""
        # Generate white noise
        white = np.random.randn(n_samples)
        
        # FFT
        fft = np.fft.rfft(white)
        
        # Create 1/f filter
        freqs = np.fft.rfftfreq(n_samples)
        freqs[0] = 1  # Avoid division by zero
        fft = fft / np.sqrt(freqs)
        
        # Inverse FFT
        pink = np.fft.irfft(fft, n_samples)
        
        # Normalize
        pink = pink / np.max(np.abs(pink))
        
        return pink
    
    def add_ssvep_response(self, eeg_data, target_freq, snr_db=10, start_time=2, end_time=8):
        """
        Add SSVEP response at specific frequency
        
        Args:
            eeg_data: Base EEG data
            target_freq: SSVEP frequency to embed
            snr_db: Signal-to-noise ratio in dB
            start_time: When SSVEP starts (seconds)
            end_time: When SSVEP ends (seconds)
        """
        # Convert SNR from dB
        snr_linear = 10 ** (snr_db / 10)
        
        # Calculate SSVEP amplitude based on noise power
        start_idx = int(start_time * self.fs)
        end_idx = int(end_time * self.fs)
        
        for ch in range(self.n_channels):
            # Stronger response in occipital channels (O1, O2, Oz)
            if 'O' in self.channel_names[ch]:
                amp_scale = 1.0
            elif 'P' in self.channel_names[ch]:
                amp_scale = 0.6  # Parietal shows moderate response
            else:
                amp_scale = 0.3  # Central shows weak response
            
            # Calculate noise power
            noise_power = np.var(eeg_data[ch, start_idx:end_idx])
            
            # Calculate SSVEP amplitude
            ssvep_amp = np.sqrt(noise_power * snr_linear) * amp_scale
            
            # Generate SSVEP signal with harmonics
            ssvep = np.zeros(len(self.t))
            
            # Fundamental frequency
            ssvep[start_idx:end_idx] = ssvep_amp * np.sin(
                2 * np.pi * target_freq * self.t[start_idx:end_idx]
            )
            
            # Add second harmonic (weaker)
            ssvep[start_idx:end_idx] += 0.3 * ssvep_amp * np.sin(
                2 * np.pi * 2 * target_freq * self.t[start_idx:end_idx]
            )
            
            # Add to EEG data
            eeg_data[ch] += ssvep
        
        return eeg_data
    
    def add_artifacts(self, eeg_data, blink_times=[3, 5, 7]):
        """Add realistic artifacts (blinks, muscle)"""
        for blink_time in blink_times:
            blink_idx = int(blink_time * self.fs)
            blink_duration = int(0.3 * self.fs)  # 300ms blink
            
            if blink_idx + blink_duration < len(self.t):
                # Blink artifact (mainly in frontal channels, but affects all)
                for ch in range(self.n_channels):
                    blink_amp = 50 if ch < 2 else 20  # Stronger in frontal
                    blink_signal = blink_amp * signal.windows.hann(blink_duration)
                    eeg_data[ch, blink_idx:blink_idx+blink_duration] += blink_signal
        
        return eeg_data
    
    def simulate_trial(self, command='left', snr_db=10, add_blinks=True):
        """
        Simulate a complete EEG trial with SSVEP response
        
        Args:
            command: Which command to simulate ('left', 'right', 'up', 'down')
            snr_db: Signal-to-noise ratio
            add_blinks: Whether to add blink artifacts
        """
        # Generate base EEG
        eeg_data = self.generate_base_rhythms()
        
        # Add SSVEP response for the command
        if command in self.ssvep_freqs:
            target_freq = self.ssvep_freqs[command]
            eeg_data = self.add_ssvep_response(eeg_data, target_freq, snr_db)
            print(f"Added SSVEP at {target_freq} Hz for command '{command}'")
        
        # Add artifacts
        if add_blinks:
            eeg_data = self.add_artifacts(eeg_data)
        
        # Add 60 Hz powerline noise
        for ch in range(self.n_channels):
            powerline = 2 * np.sin(2 * np.pi * 60 * self.t)
            eeg_data[ch] += powerline
        
        return eeg_data
    
    def plot_eeg(self, eeg_data, title="Simulated EEG Signal"):
        """Plot EEG data"""
        fig, axes = plt.subplots(self.n_channels, 1, figsize=(12, 10), sharex=True)
        
        if self.n_channels == 1:
            axes = [axes]
        
        for ch in range(self.n_channels):
            axes[ch].plot(self.t, eeg_data[ch], 'b-', linewidth=0.5)
            axes[ch].set_ylabel(f'{self.channel_names[ch]}\n(μV)')
            axes[ch].grid(True, alpha=0.3)
            axes[ch].set_ylim([-100, 100])
        
        axes[-1].set_xlabel('Time (s)')
        axes[0].set_title(title)
        plt.tight_layout()
        
        return fig
    
    def plot_spectrum(self, eeg_data, channel=0):
        """Plot frequency spectrum of a channel"""
        # Compute power spectral density
        freqs, psd = signal.welch(eeg_data[channel], self.fs, nperseg=1024)
        
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.semilogy(freqs, psd)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Power Spectral Density (μV²/Hz)')
        ax.set_title(f'Spectrum - Channel {self.channel_names[channel]}')
        ax.grid(True, alpha=0.3)
        ax.set_xlim([0, 50])
        
        # Mark SSVEP frequencies
        for cmd, freq in self.ssvep_freqs.items():
            ax.axvline(freq, color='r', linestyle='--', alpha=0.5, label=f'{cmd}: {freq} Hz')
        
        ax.legend()
        plt.tight_layout()
        
        return fig
    
    def save_data(self, eeg_data, command, snr_db):
        """Save EEG data to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"data/eeg_fake/eeg_sim_{command}_{snr_db}dB_{timestamp}"
        
        # Save as numpy array
        np.save(f"{filename}.npy", eeg_data)
        
        # Save metadata
        metadata = {
            'fs': self.fs,
            'duration': self.duration,
            'n_channels': self.n_channels,
            'channel_names': self.channel_names,
            'command': command,
            'snr_db': snr_db,
            'ssvep_freq': self.ssvep_freqs.get(command),
            'timestamp': timestamp
        }
        
        import json
        with open(f"{filename}_metadata.json", 'w') as f:
            json.dump(metadata, f, indent=4)
        
        print(f"Data saved to {filename}.npy")
        return filename

def main():
    """Test the EEG simulator"""
    # Create simulator
    sim = EEGSimulator(fs=256, duration=10, n_channels=8)
    
    # Generate trials for each command
    commands = ['left', 'right', 'up', 'down']
    snr_levels = [5, 10, 15]  # Different SNR levels
    
    for command in commands:
        for snr_db in snr_levels:
            print(f"\n--- Generating {command} command, SNR={snr_db}dB ---")
            
            # Simulate EEG trial
            eeg_data = sim.simulate_trial(command=command, snr_db=snr_db)
            
            # Save data
            filename = sim.save_data(eeg_data, command, snr_db)
            
            # Plot first trial of each command
            if snr_db == 10:
                # Time domain plot
                fig1 = sim.plot_eeg(eeg_data, 
                                   title=f"EEG Simulation - Command: {command} (SNR={snr_db}dB)")
                plt.savefig(f"{filename}_time.png", dpi=150, bbox_inches='tight')
                
                # Frequency domain plot (occipital channel)
                fig2 = sim.plot_spectrum(eeg_data, channel=0)  # O1 channel
                plt.savefig(f"{filename}_spectrum.png", dpi=150, bbox_inches='tight')
                
                plt.show()
    
    print("\n✅ EEG simulation complete! Check data/eeg_fake/ for saved files.")

if __name__ == "__main__":
    main()

