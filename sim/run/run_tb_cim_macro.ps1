# Intensive-CIM Simulation Script for CIM Macro Submodule
# Note: Requires iverilog and vvp in PATH

Push-Location "$PSScriptRoot\..\.."

try {
    # Ensure output, report and wave directories exist
    If (!(Test-Path "sim/out")) { New-Item -ItemType Directory -Path "sim/out" | Out-Null }
    If (!(Test-Path "sim/rpt")) { New-Item -ItemType Directory -Path "sim/rpt" | Out-Null }
    If (!(Test-Path "sim/wave")) { New-Item -ItemType Directory -Path "sim/wave" | Out-Null }

    Write-Host "Compiling CIM Macro Submodule..." -ForegroundColor Cyan

    iverilog -g2001 -o sim/out/sim_cim_macro.out `
      rtl/CIM_Macro/adder_tree_128.v rtl/CIM_Macro/cim_array.v rtl/CIM_Macro/cim_macro.v `
      sim/tb/tb_cim_macro.v

    if ($LASTEXITCODE -eq 0) {
      Write-Host "Compilation successful. Running simulation..." -ForegroundColor Green
      # Run vvp and capture output to sim/rpt/sim_cim_macro_report.txt using Tee-Object
      vvp sim/out/sim_cim_macro.out | Tee-Object -FilePath "sim/rpt/sim_cim_macro_report.txt"
      
      # Export wave to sim/wave
      If (Test-Path "tb_cim_macro.vcd") {
        Move-Item -Path "tb_cim_macro.vcd" -Destination "sim/wave/tb_cim_macro.vcd" -Force
        Write-Host "Waveform exported to sim/wave/tb_cim_macro.vcd" -ForegroundColor Cyan
      }
      
      Write-Host "`nSimulation complete. Report saved to sim/rpt/sim_cim_macro_report.txt" -ForegroundColor Cyan
    }
    else {
      Write-Host "Compilation failed." -ForegroundColor Red
    }
}
finally {
    Pop-Location
}

#powershell -ExecutionPolicy Bypass -File .\sim\run\run_tb_cim_macro.ps1
