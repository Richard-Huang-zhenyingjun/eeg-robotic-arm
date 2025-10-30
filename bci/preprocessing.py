#!/usr/bin/env python3
"""
EEG Preprocessing Pipeline
Implements filtering and artifact removal for EEG signals
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import json
import os

class EEGPreprocessor:
    def __init__(self, fs=256):
        """
        Initialize EEG Preprocessor
        
        Args:
            fs: Sampling frequency (Hz)
        """
        self.fs = fs
        self.nyquist = fs / 2
        
        # Filter parameters
        self.bandpass_low = 8.0   # Hz - removes slow drifts
        self.bandpass_high = 40.0 # Hz - removes high-freq noise
        self.notch_freq = 60.0    # Hz - powerline frequency
        self.notch_q = 30         # Quality factor for notch filter
        
        print(f"Preprocessor initialized (fs={fs} Hz)")
        print(f"Bandpass: {self.bandpass_low}-{self.bandpass_high} Hz")
        print(f"Notch: {self.notch_freq} Hz")
    
    def design_bandpass_filter(self, order=4):
        """
        Design Butterworth bandpass filter
        
        Args:
            order: Filter order
        Returns:
            b, a: Filter coefficients
        """
        low = self.bandpass_low / self.nyquist
        high = self.bandpass_high / self.nyquist
        
        b, a = signal.butter(order, [low, high], btype='band')
        return b, a
    
    def design_notch_filter(self):
        """
        Design notch filter for powerline noise removal
        
        Returns:
            b, a: Filter coefficients
        """
        w0 = self.notch_freq / self.nyquist
        b, a = signal.iirnotch(w0, self.notch_q)
        return b, a
    
    def apply_bandpass(self, data):
        """
        Apply bandpass filter to EEG data
        
        Args:
            data: EEG data (channels x samples)
        Returns:
            Filtered data
        """
        b, a = self.design_bandpass_filter()
        
        # Apply filter to each channel
        filtered = np.zeros_like(data)
        for ch in range(data.shape[0]):
            # Use filtfilt for zero-phase filtering
            filtered[ch] = signal.filtfilt(b, a, data[ch])
        
        return filtered
    
    def apply_notch(self, data):
        """
        Apply notch filter to remove powerline noise
        
        Args:
            data: EEG data (channels x samples)
        Returns:
            Filtered data
        """
        b, a = self.design_notch_filter()
        
        # Apply filter to each channel
        filtered = np.zeros_like(data)
        for ch in range(data.shape[0]):
            filtered[ch] = signal.filtfilt(b, a, data[ch])
        
        # Remove 2nd harmonic (120 Hz) if in range
        if 120 < self.nyquist:
            w0_2nd = 120 / self.nyquist
            b2, a2 = signal.iirnotch(w0_2nd, self.notch_q)
            for ch in range(data.shape[0]):
                filtered[ch] = signal.filtfilt(b2, a2, filtered[ch])
        
        return filtered
    
    def remove_baseline(self, data):
        """
        Remove baseline drift using high-pass filtering
        
        Args:
            data: EEG data (channels x samples)
        Returns:
            Baseline-corrected data
        """
        # Design high-pass filter (0.5 Hz cutoff)
        b, a = signal.butter(4, 0.5 / self.nyquist, btype='high')
        
        filtered = np.zeros_like(data)
        for ch in range(data.shape[0]):
            filtered[ch] = signal.filtfilt(b, a, data[ch])
        
        return filtered
    
    def detect_bad_channels(self, data, z_threshold=5):
        """
        Detect bad channels based on variance
        
        Args:
            data: EEG data (channels x samples)
            z_threshold: Z-score threshold for outlier detection
        Returns:
            List of bad channel indices
        """
        # Calculate variance for each channel
        variances = np.var(data, axis=1)
        
        # Calculate z-scores
        mean_var = np.mean(variances)
        std_var = np.std(variances)
        z_scores = np.abs((variances - mean_var) / std_var)
        
        # Find outliers
        bad_channels = np.where(z_scores > z_threshold)[0]
        
        if len(bad_channels) > 0:
            print(f"Bad channels detected: {bad_channels}")
        
        return bad_channels
    
    def interpolate_bad_channels(self, data, bad_channels):
        """
        Interpolate bad channels using neighboring channels
        
        Args:
            data: EEG data (channels x samples)
            bad_channels: List of bad channel indices
        Returns:
            Data with interpolated channels
        """
        interpolated = data.copy()
        
        for bad_ch in bad_channels:
            # Find nearest good channels
            good_channels = [ch for ch in range(data.shape[0]) if ch not in bad_channels]
            
            if len(good_channels) > 0:
                # Simple average interpolation
                interpolated[bad_ch] = np.mean(data[good_channels], axis=0)
                print(f"Interpolated channel {bad_ch}")
        
        return interpolated
    
    def preprocess(self, raw_data, remove_baseline_drift=True, interpolate_bad=True):
        """
        Complete preprocessing pipeline
        
        Args:
            raw_data: Raw EEG data (channels x samples)
            remove_baseline_drift: Whether to remove baseline drift
            interpolate_bad: Whether to interpolate bad channels
        Returns:
            Preprocessed data
        """
        print("\n--- Starting preprocessing ---")
        data = raw_data.copy()
        
        # Step 1: Detect and interpolate bad channels
        if interpolate_bad:
            bad_channels = self.detect_bad_channels(data)
            if len(bad_channels) > 0:
                data = self.interpolate_bad_channels(data, bad_channels)
        
        # Step 2: Remove baseline drift
        if remove_baseline_drift:
            print("Removing baseline drift...")
            data = self.remove_baseline(data)
        
        # Step 3: Apply notch filter
        print("Applying notch filter (60 Hz)...")
        data = self.apply_notch(data)
        
        # Step 4: Apply bandpass filter
        print(f"Applying bandpass filter ({self.bandpass_low}-{self.bandpass_high} Hz)...")
        data = self.apply_bandpass(data)
        
        print("✅ Preprocessing complete\n")
        return data
    
    def plot_comparison(self, raw_data, filtered_data, channel=0, time_range=[2, 4]):
        """
        Plot before/after filtering comparison
        
        Args:
            raw_data: Original data
            filtered_data: Filtered data
            channel: Channel index to plot
            time_range: Time range to plot [start, end] in seconds
        """
        # Time vector
        t = np.arange(raw_data.shape[1]) / self.fs
        
        # Time indices
        idx_start = int(time_range[0] * self.fs)
        idx_end = int(time_range[1] * self.fs)
        
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        
        # Time domain comparison
        axes[0, 0].plot(t[idx_start:idx_end], raw_data[channel, idx_start:idx_end], 'b-', linewidth=0.5)
        axes[0, 0].set_title(f'Raw Signal - Channel {channel}')
        axes[0, 0].set_ylabel('Amplitude (μV)')
        axes[0, 0].grid(True, alpha=0.3)
        
        axes[0, 1].plot(t[idx_start:idx_end], filtered_data[channel, idx_start:idx_end], 'g-', linewidth=0.5)
        axes[0, 1].set_title(f'Filtered Signal - Channel {channel}')
        axes[0, 1].set_ylabel('Amplitude (μV)')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Frequency domain comparison
        freqs_raw, psd_raw = signal.welch(raw_data[channel], self.fs, nperseg=1024)
        freqs_filt, psd_filt = signal.welch(filtered_data[channel], self.fs, nperseg=1024)
        
        axes[1, 0].semilogy(freqs_raw, psd_raw)
        axes[1, 0].set_title('Raw Signal Spectrum')
        axes[1, 0].set_xlabel('Frequency (Hz)')
        axes[1, 0].set_ylabel('PSD (μV²/Hz)')
        axes[1, 0].set_xlim([0, 100])
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].axvline(60, color='r', linestyle='--', alpha=0.5, label='60 Hz')
        axes[1, 0].legend()
        
        axes[1, 1].semilogy(freqs_filt, psd_filt)
        axes[1, 1].set_title('Filtered Signal Spectrum')
        axes[1, 1].set_xlabel('Frequency (Hz)')
        axes[1, 1].set_ylabel('PSD (μV²/Hz)')
        axes[1, 1].set_xlim([0, 100])
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].axvspan(self.bandpass_low, self.bandpass_high, alpha=0.2, color='green', label='Passband')
        axes[1, 1].legend()
        
        # Filter frequency response
        b_bp, a_bp = self.design_bandpass_filter()
        w_bp, h_bp = signal.freqz(b_bp, a_bp, fs=self.fs)
        
        b_notch, a_notch = self.design_notch_filter()
        w_notch, h_notch = signal.freqz(b_notch, a_notch, fs=self.fs)
        
        axes[2, 0].plot(w_bp, 20 * np.log10(abs(h_bp)), 'b-', label='Bandpass')
        axes[2, 0].set_title('Bandpass Filter Response')
        axes[2, 0].set_xlabel('Frequency (Hz)')
        axes[2, 0].set_ylabel('Magnitude (dB)')
        axes[2, 0].set_xlim([0, 50])
        axes[2, 0].grid(True, alpha=0.3)
        axes[2, 0].axvline(self.bandpass_low, color='r', linestyle='--', alpha=0.5)
        axes[2, 0].axvline(self.bandpass_high, color='r', linestyle='--', alpha=0.5)
        
        axes[2, 1].plot(w_notch, 20 * np.log10(abs(h_notch)), 'r-', label='Notch')
        axes[2, 1].set_title('Notch Filter Response')
        axes[2, 1].set_xlabel('Frequency (Hz)')
        axes[2, 1].set_ylabel('Magnitude (dB)')
        axes[2, 1].set_xlim([50, 70])
        axes[2, 1].grid(True, alpha=0.3)
        axes[2, 1].axvline(60, color='r', linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        return fig
    
    def save_filtered_data(self, data, original_filename):
        """Save filtered data with metadata"""
        # Create output filename
        base_name = os.path.basename(original_filename).replace('.npy', '')
        output_name = f"data/eeg_fake/{base_name}_filtered.npy"
        
        # Save filtered data
        np.save(output_name, data)
        
        # Save preprocessing parameters
        params = {
            'fs': self.fs,
            'bandpass_low': self.bandpass_low,
            'bandpass_high': self.bandpass_high,
            'notch_freq': self.notch_freq,
            'notch_q': self.notch_q,
            'original_file': original_filename
        }
        
        with open(output_name.replace('.npy', '_params.json'), 'w') as f:
            json.dump(params, f, indent=4)
        
        print(f"Filtered data saved to {output_name}")
        return output_name

def test_preprocessing():
    """Test the preprocessing pipeline"""
    print("=== Testing EEG Preprocessing Pipeline ===\n")
    
    # First, check if we have simulated data
    data_dir = "data/eeg_fake"
    if not os.path.exists(data_dir):
        print(f"Creating directory: {data_dir}")
        os.makedirs(data_dir)
    
    # Find existing EEG files
    eeg_files = [f for f in os.listdir(data_dir) if f.endswith('.npy') and 'filtered' not in f]
    
    if len(eeg_files) == 0:
        print("No EEG data found. Running simulator first...")
        from eeg_sim import EEGSimulator
        
        # Generate test data
        sim = EEGSimulator(fs=256, duration=10, n_channels=8)
        eeg_data = sim.simulate_trial(command='left', snr_db=10)
        filename = sim.save_data(eeg_data, 'left', 10)
        eeg_files = [os.path.basename(filename) + '.npy']
    
    # Process first file
    test_file = os.path.join(data_dir, eeg_files[0])
    print(f"Processing: {test_file}")
    
    # Load data
    raw_data = np.load(test_file)
    print(f"Data shape: {raw_data.shape}")
    
    # Initialize preprocessor
    preprocessor = EEGPreprocessor(fs=256)
    
    # Apply preprocessing
    filtered_data = preprocessor.preprocess(raw_data)
    
    # Save filtered data
    output_file = preprocessor.save_filtered_data(filtered_data, test_file)
    
    # Plot comparison
    fig = preprocessor.plot_comparison(raw_data, filtered_data, channel=0)
    plt.savefig(output_file.replace('.npy', '_comparison.png'), dpi=150, bbox_inches='tight')
    plt.show()
    
    print("\n✅ Preprocessing test complete!")
    
    # Process all files
    print("\n--- Processing all EEG files ---")
    for eeg_file in eeg_files:
        if 'filtered' not in eeg_file:
            file_path = os.path.join(data_dir, eeg_file)
            data = np.load(file_path)
            filtered = preprocessor.preprocess(data)
            preprocessor.save_filtered_data(filtered, file_path)

if __name__ == "__main__":
    test_preprocessing()

