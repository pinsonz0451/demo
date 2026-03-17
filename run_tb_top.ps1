# Intensive-CIM Simulation Script
# Note: Requires iverilog and vvp in PATH

# Ensure output, report and wave directories exist
If (!(Test-Path "sim/out")) { New-Item -ItemType Directory -Path "sim/out" | Out-Null }
If (!(Test-Path "sim/rpt")) { New-Item -ItemType Directory -Path "sim/rpt" | Out-Null }
If (!(Test-Path "sim/wave")) { New-Item -ItemType Directory -Path "sim/wave" | Out-Null }

Write-Host "Compiling Intensive-CIM Datapath..." -ForegroundColor Cyan

iverilog -g2001 -o sim/out/sim_top.out `
  rtl/Aligner/aligner.v rtl/Aligner/aligner_channel.v `
  rtl/CIM_Macro/adder_tree_128.v rtl/CIM_Macro/cim_array.v rtl/CIM_Macro/cim_macro.v `
  rtl/Normalization/int_to_fp21.v rtl/Normalization/fp21_adder.v `
  rtl/Normalization/fp21_to_fp16.v rtl/Normalization/int_to_fp_acc.v `
  rtl/CIM_TOP.v sim/tb/tb_top.v

if ($LASTEXITCODE -eq 0) {
  Write-Host "Compilation successful. Running simulation..." -ForegroundColor Green
  # Run vvp and capture output to sim/rpt/sim_report.txt using Tee-Object
  vvp sim/out/sim_top.out | Tee-Object -FilePath "sim/rpt/sim_report.txt"
  
  # Export wave to sim/wave
  If (Test-Path "tb_top.vcd") {
    Move-Item -Path "tb_top.vcd" -Destination "sim/wave/tb_top.vcd" -Force
    Write-Host "Waveform exported to sim/wave/tb_top.vcd" -ForegroundColor Cyan
  }
  
  Write-Host "`nSimulation complete. Report saved to sim/rpt/sim_report.txt" -ForegroundColor Cyan
}
else {
  Write-Host "Compilation failed." -ForegroundColor Red
}

#powershell -ExecutionPolicy Bypass -File .\run_tb_top.ps1