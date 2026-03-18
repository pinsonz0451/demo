# Intensive-CIM Simulation Script for Normalization (Int to FP) Submodule
# Note: Requires iverilog and vvp in PATH

Push-Location "$PSScriptRoot\..\.."

try {
    # Ensure output, report and wave directories exist
    If (!(Test-Path "sim/out")) { New-Item -ItemType Directory -Path "sim/out" | Out-Null }
    If (!(Test-Path "sim/rpt")) { New-Item -ItemType Directory -Path "sim/rpt" | Out-Null }
    If (!(Test-Path "sim/wave")) { New-Item -ItemType Directory -Path "sim/wave" | Out-Null }

    Write-Host "Compiling Normalization Submodule..." -ForegroundColor Cyan

    iverilog -g2001 -o sim/out/sim_int_to_fp.out `
      rtl/Normalization/int_to_fp21.v rtl/Normalization/fp21_adder.v `
      rtl/Normalization/fp21_to_fp16.v rtl/Normalization/int_to_fp_acc.v `
      sim/tb/tb_int_to_fp_acc.v

    if ($LASTEXITCODE -eq 0) {
      Write-Host "Compilation successful. Running simulation..." -ForegroundColor Green
      # Run vvp and capture output to sim/rpt/sim_int_to_fp_report.txt using Tee-Object
      vvp sim/out/sim_int_to_fp.out | Tee-Object -FilePath "sim/rpt/sim_int_to_fp_report.txt"
      
      # Export wave to sim/wave
      If (Test-Path "tb_int_to_fp_acc.vcd") {
        Move-Item -Path "tb_int_to_fp_acc.vcd" -Destination "sim/wave/tb_int_to_fp_acc.vcd" -Force
        Write-Host "Waveform exported to sim/wave/tb_int_to_fp_acc.vcd" -ForegroundColor Cyan
      }
      
      Write-Host "`nSimulation complete. Report saved to sim/rpt/sim_int_to_fp_report.txt" -ForegroundColor Cyan
    }
    else {
      Write-Host "Compilation failed." -ForegroundColor Red
    }
}
finally {
    Pop-Location
}

#powershell -ExecutionPolicy Bypass -File .\sim\run\run_tb_int_to_fp.ps1
