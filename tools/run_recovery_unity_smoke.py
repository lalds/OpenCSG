#!/usr/bin/env python3
"""Prepare an isolated Unity project and verify the research native scene bridge.

Only explicitly selected source files are imported, never the original native
plugin or editor startup. Existing projects without this tool's marker are
refused. --prepare-only prints the Unity command.
"""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
MARKER='.recovery-smoke-project'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unity',type=Path,help='Unity editor executable; required without --prepare-only')
    parser.add_argument('--library',type=Path,required=True,help='Built Linux librealtimecsg_recovered_geometry.so')
    parser.add_argument('--project',type=Path,required=True,help='New or previously marked isolated test project')
    parser.add_argument('--unity-version',default='2022.3.62f3')
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--foundation',action='store_true',help='Test original Foundation BrushMesh data and transformed brushes')
    args=parser.parse_args()
    project=args.project.resolve(); library=args.library.resolve()
    if not library.is_file() or library.suffix!='.so': parser.error('--library must name the built Linux .so')
    if not args.prepare_only and (args.unity is None or not args.unity.is_file()): parser.error('--unity must name the editor executable')
    if project.exists() and any(project.iterdir()) and not (project/MARKER).is_file():
        parser.error('Refusing to overwrite a project without '+MARKER)
    for folder in ['Assets/Recovery','Assets/Editor','Assets/Plugins/x86_64','ProjectSettings','Packages']:
        (project/folder).mkdir(parents=True,exist_ok=True)
    (project/MARKER).write_text('Research bridge smoke project, managed by run_recovery_unity_smoke.py\n')
    for name in ['RecoveredScene.cs','RecoveryPrimitives.cs']:
        shutil.copyfile(ROOT/'native/managed'/name,project/'Assets/Recovery'/name)
    shutil.copyfile(ROOT/'native/managed/RecoveryUnityPreview.cs',project/'Assets/Editor/RecoveryUnityPreview.cs')
    if args.foundation:
        shutil.copyfile(ROOT/'native/managed/RecoveryFoundationBridge.cs',project/'Assets/Editor/RecoveryFoundationBridge.cs')
        shutil.copyfile(ROOT/'native/tests/unity_foundation_smoke.cs',project/'Assets/Editor/RecoveryFoundationSmoke.cs')
        for source in ['Plugins/API/Foundation/Editor/BrushMesh.cs',
                       'Plugins/API/Foundation/Editor/BrushMeshUtility.cs',
                       'Plugins/API/Foundation/Runtime/SurfaceLayers.cs']:
            shutil.copyfile(ROOT/source,project/'Assets/Editor'/Path(source).name)
    shutil.copyfile(library,project/'Assets/Plugins/x86_64/librealtimecsg_recovered_geometry.so')
    (project/'ProjectSettings/ProjectVersion.txt').write_text('m_EditorVersion: '+args.unity_version+'\n')
    (project/'Packages/manifest.json').write_text(json.dumps({'dependencies':{}},indent=2)+'\n')
    log=project.parent/(project.name+'.log')
    method='RealtimeCSG.Recovery.FoundationSmoke.BatchSmoke' if args.foundation else 'RealtimeCSG.Recovery.UnityPreview.BatchSmoke'
    command=[str(args.unity.resolve()) if args.unity else 'UNITY_EXECUTABLE','-batchmode','-nographics','-projectPath',str(project),
        '-executeMethod',method,'-logFile',str(log),'-quit']
    print(shlex.join(command),flush=True)
    if args.prepare_only: return
    report=project/('RecoveryFoundationReport.json' if args.foundation else 'RecoveryPreviewReport.json')
    if report.exists(): report.unlink()
    code=subprocess.run(command,check=False).returncode
    if code or not report.is_file(): raise SystemExit(f'Unity smoke failed (exit {code}); inspect {log}')
    result=json.loads(report.read_text())
    if result.get('nativeAbi')!=1 or abs(result.get('volume',0)-6)>1e-5:
        raise SystemExit('Unexpected smoke report: '+str(report))
    print(json.dumps(result,indent=2))

if __name__=='__main__': main()
