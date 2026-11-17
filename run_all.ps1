$ErrorActionPreference = 'Stop'
python .\run_all.py
if ($LASTEXITCODE -ne 0) { throw 'Artifact reproduction failed.' }
