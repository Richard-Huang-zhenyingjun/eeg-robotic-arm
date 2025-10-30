# BCI Module

## Components
- `preprocessing.py` - Signal filtering pipeline
- `decoder.py` - CCA-based SSVEP decoder

## Usage
```python
from bci.preprocessing import EEGPreprocessor
from bci.decoder import CCADecoder

# Preprocess signals
preprocessor = EEGPreprocessor(fs=256)
filtered_data = preprocessor.preprocess(raw_data)

# Decode SSVEP
decoder = CCADecoder(fs=256)
command, scores = decoder.detect_frequency(filtered_data)
```
