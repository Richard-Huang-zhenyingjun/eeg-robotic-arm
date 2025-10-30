#!/usr/bin/env python3
"""
CCA-based SSVEP Decoder for BCI Control
Implements Canonical Correlation Analysis for frequency detection
"""

import numpy as np
from scipy import signal, linalg
from scipy.stats import pearsonr
import os
import json
import matplotlib.pyplot as plt

class CCADecoder:
    def __init__(self, fs=256, n_harmonics=2):
        """
        Initialize CCA-based SSVEP decoder
        
        Args:
            fs: Sampling frequency (Hz)
            n_harmonics: Number of harmonics to include in reference signals
        """
        self.fs = fs
        self.n_harmonics = n_harmonics
        
        # Target SSVEP frequencies for robot commands
        self.target_freqs = {
            'left': 10.0,
            'right': 12.0,
            'up': 15.0,
            'down': 8.5
        }
        
        # Window parameters for online detection
        self.window_length = 4.0  # seconds
        self.window_overlap = 0.5  # 50% overlap
        
        print(f"CCA Decoder initialized")
        print(f"Target frequencies: {self.target_freqs}")
        print(f"Harmonics: {n_harmonics}")
    
    def generate_reference_signals(self, freq, n_samples):
        """
        Generate sine-cosine reference signals for CCA
        
        Args:
            freq: Target frequency (Hz)
            n_samples: Number of samples
        
        Returns:
            Reference signal matrix (2*n_harmonics x n_samples)
        """
        t = np.arange(n_samples) / self.fs
        references = []
        
        for harmonic in range(1, self.n_harmonics + 1):
            # Sin and cos for each harmonic
            references.append(np.sin(2 * np.pi * harmonic * freq * t))
            references.append(np.cos(2 * np.pi * harmonic * freq * t))
        
        return np.array(references)
    
    def canonical_correlation(self, X, Y):
        """
        Compute canonical correlation between two datasets
        
        Args:
            X: Data matrix 1 (variables x observations)
            Y: Data matrix 2 (variables x observations)
        
        Returns:
            Maximum canonical correlation coefficient
        """
        # Center the data
        X = X - np.mean(X, axis=1, keepdims=True)
        Y = Y - np.mean(Y, axis=1, keepdims=True)
        
        # Compute covariance matrices
        n = X.shape[1]
        
        # Within-set covariances
        Cxx = (X @ X.T) / (n - 1)
        Cyy = (Y @ Y.T) / (n - 1)
        
        # Between-sets covariance
        Cxy = (X @ Y.T) / (n - 1)
        Cyx = Cxy.T
        
        # Add regularization to avoid singular matrices
        reg = 1e-8
        Cxx = Cxx + reg * np.eye(Cxx.shape[0])
        Cyy = Cyy + reg * np.eye(Cyy.shape[0])
        
        # Solve generalized eigenvalue problem
        try:
            # Method 1: Direct computation
            inv_Cxx = np.linalg.inv(Cxx)
            inv_Cyy = np.linalg.inv(Cyy)
            
            # Compute canonical correlation matrix
            M = inv_Cxx @ Cxy @ inv_Cyy @ Cyx
            
            # Get eigenvalues (squared canonical correlations)
            eigenvalues = np.linalg.eigvalsh(M)
            
            # Maximum canonical correlation
            max_corr = np.sqrt(np.max(np.abs(eigenvalues)))
            
        except np.linalg.LinAlgError:
            # Fallback: use simple correlation
            max_corr = 0
            for i in range(X.shape[0]):
                for j in range(Y.shape[0]):
                    corr, _ = pearsonr(X[i], Y[j])
                    max_corr = max(max_corr, abs(corr))
        
        return max_corr
    
    def detect_frequency(self, eeg_segment):
        """
        Detect SSVEP frequency in EEG segment using CCA
        
        Args:
            eeg_segment: EEG data segment (channels x samples)
        
        Returns:
            detected_command: Detected command ('left', 'right', 'up', 'down')
            correlation_scores: CCA scores for each frequency
        """
        n_samples = eeg_segment.shape[1]
        correlation_scores = {}
        
        # Compute CCA for each target frequency
        for command, freq in self.target_freqs.items():
            # Generate reference signals
            ref_signals = self.generate_reference_signals(freq, n_samples)
            
            # Compute canonical correlation
            cca_score = self.canonical_correlation(eeg_segment, ref_signals)
            correlation_scores[command] = cca_score
        
        # Find frequency with highest correlation
        detected_command = max(correlation_scores, key=correlation_scores.get)
        
        return detected_command, correlation_scores
    
    def classify_trial(self, eeg_data, true_command=None):
        """
        Classify a complete EEG trial
        
        Args:
            eeg_data: Complete EEG trial (channels x samples)
            true_command: Ground truth command for accuracy calculation
        
        Returns:
            prediction: Predicted command
            confidence: Confidence score
            is_correct: Whether prediction matches truth (if provided)
        """
        # Use middle segment of trial (avoid onset/offset)
        total_samples = eeg_data.shape[1]
        segment_samples = int(self.window_length * self.fs)
        
        # Extract middle segment
        start_idx = (total_samples - segment_samples) // 2
        end_idx = start_idx + segment_samples
        segment = eeg_data[:, start_idx:end_idx]
        
        # Detect frequency
        prediction, scores = self.detect_frequency(segment)
        
        # Calculate confidence (ratio of best to second-best)
        sorted_scores = sorted(scores.values(), reverse=True)
        if len(sorted_scores) > 1 and sorted_scores[1] > 0:
            confidence = sorted_scores[0] / sorted_scores[1]
        else:
            confidence = sorted_scores[0]
        
        # Check accuracy if ground truth provided
        is_correct = (prediction == true_command) if true_command else None
        
        return prediction, confidence, is_correct, scores
    
    def evaluate_dataset(self, data_dir='data/eeg_fake'):
        """
        Evaluate decoder on all filtered EEG files
        
        Args:
            data_dir: Directory containing filtered EEG files
        
        Returns:
            results: Dictionary with evaluation metrics
        """
        results = {
            'total_trials': 0,
            'correct': 0,
            'confusion_matrix': {cmd: {pred: 0 for pred in self.target_freqs.keys()} 
                               for cmd in self.target_freqs.keys()},
            'per_command_accuracy': {},
            'per_snr_accuracy': {},
            'all_predictions': []
        }
        
        # Find all filtered EEG files
        filtered_files = [f for f in os.listdir(data_dir) 
                         if f.endswith('_filtered.npy')]
        
        print(f"\nEvaluating {len(filtered_files)} trials...")
        print("-" * 50)
        
        for file in filtered_files:
            # Load EEG data
            eeg_data = np.load(os.path.join(data_dir, file))
            
            # Parse metadata from filename
            # Format: eeg_sim_COMMAND_SNRdB_timestamp_filtered.npy
            parts = file.split('_')
            true_command = parts[2]  # Extract command
            snr = parts[3]  # Extract SNR level
            
            # Skip if not a valid command
            if true_command not in self.target_freqs.keys():
                continue
            
            # Classify trial
            prediction, confidence, is_correct, scores = self.classify_trial(
                eeg_data, true_command
            )
            
            # Update results
            results['total_trials'] += 1
            if is_correct:
                results['correct'] += 1
            
            results['confusion_matrix'][true_command][prediction] += 1
            
            # Store prediction details
            results['all_predictions'].append({
                'file': file,
                'true': true_command,
                'predicted': prediction,
                'correct': is_correct,
                'confidence': confidence,
                'scores': scores,
                'snr': snr
            })
            
            # Print trial result
            status = "✓" if is_correct else "✗"
            print(f"{status} {file:40} True: {true_command:5} → Pred: {prediction:5} "
                  f"(conf: {confidence:.2f})")
        
        # Calculate accuracies
        results['overall_accuracy'] = results['correct'] / results['total_trials'] * 100
        
        # Per-command accuracy
        for cmd in self.target_freqs.keys():
            total = sum(results['confusion_matrix'][cmd].values())
            if total > 0:
                correct = results['confusion_matrix'][cmd][cmd]
                results['per_command_accuracy'][cmd] = correct / total * 100
        
        # Per-SNR accuracy
        for prediction in results['all_predictions']:
            snr = prediction['snr']
            if snr not in results['per_snr_accuracy']:
                results['per_snr_accuracy'][snr] = {'correct': 0, 'total': 0}
            
            results['per_snr_accuracy'][snr]['total'] += 1
            if prediction['correct']:
                results['per_snr_accuracy'][snr]['correct'] += 1
        
        # Calculate SNR percentages
        for snr in results['per_snr_accuracy']:
            data = results['per_snr_accuracy'][snr]
            data['accuracy'] = data['correct'] / data['total'] * 100
        
        return results
    
    def plot_confusion_matrix(self, results):
        """Plot confusion matrix"""
        commands = list(self.target_freqs.keys())
        matrix = np.zeros((len(commands), len(commands)))
        
        for i, true_cmd in enumerate(commands):
            for j, pred_cmd in enumerate(commands):
                matrix[i, j] = results['confusion_matrix'][true_cmd][pred_cmd]
        
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(matrix, cmap='Blues')
        
        # Labels
        ax.set_xticks(np.arange(len(commands)))
        ax.set_yticks(np.arange(len(commands)))
        ax.set_xticklabels(commands)
        ax.set_yticklabels(commands)
        ax.set_xlabel('Predicted Command')
        ax.set_ylabel('True Command')
        ax.set_title('Confusion Matrix - CCA Decoder')
        
        # Add numbers
        for i in range(len(commands)):
            for j in range(len(commands)):
                text = ax.text(j, i, int(matrix[i, j]),
                             ha="center", va="center", color="black")
        
        plt.colorbar(im)
        plt.tight_layout()
        return fig
    
    def plot_scores_distribution(self, results):
        """Plot CCA scores distribution"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        axes = axes.flatten()
        
        commands = list(self.target_freqs.keys())
        
        for idx, cmd in enumerate(commands):
            # Collect scores for this true command
            cmd_scores = {freq: [] for freq in commands}
            
            for pred in results['all_predictions']:
                if pred['true'] == cmd:
                    for freq_cmd, score in pred['scores'].items():
                        cmd_scores[freq_cmd].append(score)
            
            # Plot distribution
            ax = axes[idx]
            positions = range(len(commands))
            box_data = [cmd_scores[c] for c in commands]
            
            bp = ax.boxplot(box_data, positions=positions, labels=commands)
            ax.set_title(f'True Command: {cmd.upper()}')
            ax.set_xlabel('Target Frequency')
            ax.set_ylabel('CCA Score')
            ax.grid(True, alpha=0.3)
            
            # Highlight correct frequency
            correct_idx = commands.index(cmd)
            ax.axvspan(correct_idx - 0.25, correct_idx + 0.25, 
                      alpha=0.2, color='green')
        
        plt.suptitle('CCA Score Distributions by True Command')
        plt.tight_layout()
        return fig
    
    def save_results(self, results, output_dir='data/eeg_fake'):
        """Save evaluation results"""
        # Save as JSON
        results_copy = results.copy()
        # Convert numpy values to Python types for JSON serialization
        results_copy['all_predictions'] = [
            {k: float(v) if isinstance(v, np.floating) else v 
             for k, v in pred.items()}
            for pred in results_copy['all_predictions']
        ]
        
        with open(os.path.join(output_dir, 'cca_results.json'), 'w') as f:
            json.dump(results_copy, f, indent=4, default=str)
        
        print(f"\nResults saved to {output_dir}/cca_results.json")

def main():
    """Test the CCA decoder"""
    print("=== CCA-based SSVEP Decoder ===\n")
    
    # Initialize decoder
    decoder = CCADecoder(fs=256, n_harmonics=2)
    
    # Evaluate on dataset
    results = decoder.evaluate_dataset('data/eeg_fake')
    
    # Print summary
    print("\n" + "=" * 50)
    print("EVALUATION SUMMARY")
    print("=" * 50)
    print(f"Overall Accuracy: {results['overall_accuracy']:.1f}% "
          f"({results['correct']}/{results['total_trials']})")
    
    print("\nPer-Command Accuracy:")
    for cmd, acc in results['per_command_accuracy'].items():
        print(f"  {cmd:8} {acc:.1f}%")
    
    print("\nPer-SNR Accuracy:")
    for snr in sorted(results['per_snr_accuracy'].keys()):
        data = results['per_snr_accuracy'][snr]
        print(f"  {snr:8} {data['accuracy']:.1f}% ({data['correct']}/{data['total']})")
    
    # Plot results
    fig1 = decoder.plot_confusion_matrix(results)
    plt.savefig('data/eeg_fake/confusion_matrix.png', dpi=150, bbox_inches='tight')
    
    fig2 = decoder.plot_scores_distribution(results)
    plt.savefig('data/eeg_fake/cca_scores.png', dpi=150, bbox_inches='tight')
    
    plt.show()
    
    # Save results
    decoder.save_results(results)
    
    print("\n✅ CCA decoder evaluation complete!")

if __name__ == "__main__":
    main()

