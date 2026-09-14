# AoA Error FAR/MDR Reproduction Package

This package contains the MATLAB code, saved simulation data, and figure outputs for the Gaussian AoA-error experiment with `M = 64, 128`, `SNR = 0:10 dB`, and AoA error variance `0.1 deg^2`.

## Folder contents

- `code/FDR_TII_mainR1_SNR_AoA.m`: generates the AoA-error simulation data.
- `code/HuituTII_FAR_MDR_AoA_SNR.m`: generates one FAR figure and three MDR figures from the saved data.
- `code/PDF2.m`, `code/GateDnTII2.m`: local MATLAB dependencies.
- `data/FDR_AoA_SNR_M64_128_Var0p1.mat`: saved simulation data used by the plotting script.
- `results/`: editable MATLAB FIG files and vector PDF outputs.

## Result mapping

- `FAR_AoA_SNR_M64_128_Var0p1_NoAttack_Modeling`: FAR under no attack.
- `MDR_AoA_SNR_M64_128_Var0p1_PSA`: MDR under PSA.
- `MDR_AoA_SNR_M64_128_Var0p1_UPM`: MDR under DCISA (`UPM` is retained in the original filename).
- `MDR_AoA_SNR_M64_128_Var0p1_UPM_PSA`: MDR under PSA+DCISA.

## Requirements

- MATLAB
- Communications Toolbox (`pskmod` is used by the simulation script)

The source files and result filenames are copied without modification.
