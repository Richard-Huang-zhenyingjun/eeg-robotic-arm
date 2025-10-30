#!/usr/bin/env python3
"""
EEG Analysis and Visualization
Comprehensive analysis of SSVEP signals and decoder performance
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os
import json
import pandas as pd
from matplotlib.gridspec import GridSpec

class EEGAnalyzer:
    def __init__(self, fs=256):
        """Initialize EEG analyzer"""
        self.fs = fs
        self.target_freqs = {
            'left': 10.0,
            'right': 12.0,
            'up': 15.0,
            'down': 8.5
        }
        
        # Create output directory for images
        os.makedirs('docs/images', exist_ok=True)
        
    def analyze_frequency_spectrum(self, data_dir='data/eeg_fake'):
        """Analyze frequency spectrum of all EEG files"""
        filtered_files = [f for f in os.listdir(data_dir) 
                         if f.endswith('_filtered.npy')]
        
        # Group files by command
        command_files = {cmd: [] for cmd in self.target_freqs.keys()}
        
        for file in filtered_files:
            parts = file.split('_')
            if len(parts) > 2:
                command = parts[2]
                if command in command_files:
                    command_files[command].append(file)
        
        # Create subplot for each command
        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(2, 2, figure=fig)
        
        for idx, (command, files) in enumerate(command_files.items()):
            ax = fig.add_subplot(gs[idx // 2, idx % 2])
            
            # Average spectrum across trials
            all_psds = []
            
            for file in files:
                eeg_data = np.load(os.path.join(data_dir, file))
                
                # Use occipital channel (best for SSVEP)
                channel = 0  # O1
                
                # Compute PSD
                freqs, psd = signal.welch(eeg_data[channel], 
                                         self.fs, 
                                         nperseg=1024)
                all_psds.append(psd)
            
            if all_psds:
                # Average PSD
                mean_psd = np.mean(all_psds, axis=0)
                std_psd = np.std(all_psds, axis=0)
                
                # Plot
                ax.semilogy(freqs, mean_psd, 'b-', linewidth=2, 
                           label=f'Mean (n={len(files)})')
                ax.fill_between(freqs, 
                               mean_psd - std_psd, 
                               mean_psd + std_psd,
                               alpha=0.3, color='blue')
                
                # Mark target frequency
                target_freq = self.target_freqs[command]
                ax.axvline(target_freq, color='red', linestyle='--', 
                          linewidth=2, label=f'Target: {target_freq} Hz')
                
                # Mark harmonics
                ax.axvline(target_freq * 2, color='orange', linestyle=':', 
                          alpha=0.7, label='2nd harmonic')
                
                ax.set_xlim([5, 30])
                ax.set_xlabel('Frequency (Hz)')
                ax.set_ylabel('PSD (μV²/Hz)')
                ax.set_title(f'Command: {command.upper()}')
                ax.grid(True, alpha=0.3)
                ax.legend()
        
        plt.suptitle('Frequency Spectrum Analysis by Command', fontsize=16)
        plt.tight_layout()
        
        # Save figure
        plt.savefig('docs/images/frequency_spectrum_analysis.png', 
                   dpi=150, bbox_inches='tight')
        return fig
    
    def analyze_snr_performance(self, results):
        """Analyze decoder performance vs SNR"""
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        
        # Extract SNR levels and accuracies
        snr_data = results['per_snr_accuracy']
        snr_levels = []
        accuracies = []
        
        for snr_str in sorted(snr_data.keys()):
            # Extract numeric SNR value
            snr_val = int(snr_str.replace('dB', ''))
            snr_levels.append(snr_val)
            accuracies.append(snr_data[snr_str]['accuracy'])
        
        # Plot 1: Accuracy vs SNR
        axes[0].plot(snr_levels, accuracies, 'bo-', linewidth=2, markersize=10)
        axes[0].set_xlabel('SNR (dB)')
        axes[0].set_ylabel('Accuracy (%)')
        axes[0].set_title('Decoder Accuracy vs Signal-to-Noise Ratio')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_ylim([0, 105])
        
        # Add value labels
        for snr, acc in zip(snr_levels, accuracies):
            axes[0].text(snr, acc + 2, f'{acc:.1f}%', 
                        ha='center', fontsize=10)
        
        # Plot 2: Per-command accuracy by SNR
        commands = list(self.target_freqs.keys())
        snr_command_acc = {snr: {cmd: {'correct': 0, 'total': 0} 
                                 for cmd in commands} 
                          for snr in snr_levels}
        
        # Collect per-command, per-SNR data
        for pred in results['all_predictions']:
            snr = int(pred['snr'].replace('dB', ''))
            cmd = pred['true']
            snr_command_acc[snr][cmd]['total'] += 1
            if pred['correct']:
                snr_command_acc[snr][cmd]['correct'] += 1
        
        # Plot lines for each command
        for cmd in commands:
            cmd_accuracies = []
            for snr in snr_levels:
                if snr_command_acc[snr][cmd]['total'] > 0:
                    acc = (snr_command_acc[snr][cmd]['correct'] / 
                          snr_command_acc[snr][cmd]['total'] * 100)
                else:
                    acc = 0
                cmd_accuracies.append(acc)
            
            axes[1].plot(snr_levels, cmd_accuracies, 'o-', 
                        label=cmd.upper(), linewidth=2, markersize=8)
        
        axes[1].set_xlabel('SNR (dB)')
        axes[1].set_ylabel('Accuracy (%)')
        axes[1].set_title('Per-Command Accuracy vs SNR')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        axes[1].set_ylim([0, 105])
        
        plt.tight_layout()
        plt.savefig('docs/images/snr_performance.png', 
                   dpi=150, bbox_inches='tight')
        return fig
    
    def analyze_cca_scores(self, results):
        """Analyze CCA score patterns"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Collect score differences
        score_diffs = []
        confidences = []
        
        for pred in results['all_predictions']:
            scores = pred['scores']
            true_cmd = pred['true']
            
            # Get score for true frequency
            true_score = scores[true_cmd]
            
            # Get max score from other frequencies
            other_scores = [score for cmd, score in scores.items() 
                          if cmd != true_cmd]
            max_other = max(other_scores) if other_scores else 0
            
            score_diffs.append(true_score - max_other)
            confidences.append(pred['confidence'])
        
        # Plot 1: Score difference distribution
        axes[0, 0].hist(score_diffs, bins=30, edgecolor='black', alpha=0.7)
        axes[0, 0].axvline(0, color='red', linestyle='--', linewidth=2)
        axes[0, 0].set_xlabel('Score Difference (True - Max Other)')
        axes[0, 0].set_ylabel('Count')
        axes[0, 0].set_title('CCA Score Separation')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Plot 2: Confidence distribution
        axes[0, 1].hist(confidences, bins=30, edgecolor='black', alpha=0.7)
        axes[0, 1].set_xlabel('Confidence Score')
        axes[0, 1].set_ylabel('Count')
        axes[0, 1].set_title('Decoder Confidence Distribution')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: Score difference vs correctness
        correct_diffs = [score_diffs[i] for i, p in enumerate(results['all_predictions']) 
                        if p['correct']]
        incorrect_diffs = [score_diffs[i] for i, p in enumerate(results['all_predictions']) 
                          if not p['correct']]
        
        axes[1, 0].boxplot([correct_diffs, incorrect_diffs], 
                          labels=['Correct', 'Incorrect'])
        axes[1, 0].set_ylabel('Score Difference')
        axes[1, 0].set_title('Score Separation by Correctness')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: Average scores per frequency
        avg_scores = {cmd: [] for cmd in self.target_freqs.keys()}
        
        for pred in results['all_predictions']:
            for cmd, score in pred['scores'].items():
                avg_scores[cmd].append(score)
        
        positions = range(len(avg_scores))
        commands = list(avg_scores.keys())
        
        bp = axes[1, 1].boxplot([avg_scores[cmd] for cmd in commands],
                               positions=positions,
                               labels=[cmd.upper() for cmd in commands])
        axes[1, 1].set_xlabel('Target Command')
        axes[1, 1].set_ylabel('CCA Score')
        axes[1, 1].set_title('CCA Score Distribution by Target')
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.suptitle('CCA Score Analysis', fontsize=16)
        plt.tight_layout()
        
        plt.savefig('docs/images/cca_score_analysis.png', 
                   dpi=150, bbox_inches='tight')
        return fig
    
    def create_summary_report(self, results):
        """Create summary report with all metrics"""
        fig = plt.figure(figsize=(14, 10))
        
        # Create text summary
        summary_text = f"""
EEG DECODER EVALUATION SUMMARY
{'='*50}

Overall Performance:
  • Total Trials: {results['total_trials']}
  • Correct Classifications: {results['correct']}
  • Overall Accuracy: {results['overall_accuracy']:.1f}%

Per-Command Accuracy:
"""
        for cmd, acc in results['per_command_accuracy'].items():
            freq = self.target_freqs[cmd]
            summary_text += f"  • {cmd.upper():8} ({freq:5.1f} Hz): {acc:5.1f}%\n"
        
        summary_text += "\nPer-SNR Accuracy:\n"
        for snr in sorted(results['per_snr_accuracy'].keys()):
            data = results['per_snr_accuracy'][snr]
            summary_text += f"  • {snr:8}: {data['accuracy']:5.1f}% ({data['correct']}/{data['total']})\n"
        
        # Add confusion matrix data
        summary_text += "\nConfusion Matrix:\n"
        summary_text += "         Predicted:\n"
        summary_text += "True     LEFT  RIGHT  UP   DOWN\n"
        
        commands = ['left', 'right', 'up', 'down']
        for true_cmd in commands:
            row = f"{true_cmd.upper():8}"
            for pred_cmd in commands:
                count = results['confusion_matrix'][true_cmd][pred_cmd]
                row += f"{count:5}"
            summary_text += row + "\n"
        
        # Display text
        ax = fig.add_subplot(111)
        ax.axis('off')
        ax.text(0.1, 0.5, summary_text, fontsize=12, 
               family='monospace', verticalalignment='center')
        
        plt.savefig('docs/images/evaluation_summary.png', 
                   dpi=150, bbox_inches='tight')
        
        # Save as text file too
        with open('docs/images/evaluation_summary.txt', 'w') as f:
            f.write(summary_text)
        
        return fig
    
    def generate_all_plots(self):
        """Generate all analysis plots"""
        print("Generating analysis plots...")
        
        # Load CCA results if available
        if os.path.exists('data/eeg_fake/cca_results.json'):
            with open('data/eeg_fake/cca_results.json', 'r') as f:
                results = json.load(f)
            
            # Generate plots
            self.analyze_frequency_spectrum()
            self.analyze_snr_performance(results)
            self.analyze_cca_scores(results)
            self.create_summary_report(results)
            
            print("✅ All plots saved to docs/images/")
        else:
            print("⚠️  Run CCA decoder first to generate results!")

def main():
    """Run complete EEG analysis"""
    analyzer = EEGAnalyzer()
    analyzer.generate_all_plots()
    
    print("\nAnalysis complete! Check docs/images/ for visualizations.")

if __name__ == "__main__":
    main()

