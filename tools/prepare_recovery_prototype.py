#!/usr/bin/env python3
"""Prepare a separate, editable Unity CSG prototype using the open research kernel."""
import argparse
from pathlib import Path
import json
import shlex
import shutil

ROOT = Path(__file__).resolve().parents[1]
MARKER = '.recovery-prototype-project'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--unity', type=Path)
    parser.add_argument('--unity-version', default='2022.3.62f3')
    args = parser.parse_args()
    project = args.project.resolve()
    library = args.library.resolve()
    if not library.is_file() or library.suffix != '.so':
        parser.error('--library must name the built Linux .so')
    if project.exists() and (not project.is_dir() or (any(project.iterdir()) and not (project / MARKER).is_file())):
        parser.error('Refusing to modify an existing project without ' + MARKER)
    for folder in ['Assets/Recovery/Foundation', 'Assets/Editor/Recovery', 'Assets/Plugins/x86_64', 'Packages', 'ProjectSettings']:
        (project / folder).mkdir(parents=True, exist_ok=True)
    (project / MARKER).write_text('Open CSG editable prototype project\n')
    for name in ['RecoveredScene.cs', 'RecoveryFoundationBridge.cs', 'RecoveryModel.cs', 'RecoveryBrush.cs']:
        shutil.copyfile(ROOT / 'native/managed' / name, project / 'Assets/Recovery' / name)
    shutil.copyfile(ROOT / 'native/managed/RecoveryPrototypeEditor.cs', project / 'Assets/Editor/Recovery/RecoveryPrototypeEditor.cs')
    shutil.copyfile(ROOT / 'native/tests/unity_prototype_smoke.cs', project / 'Assets/Editor/Recovery/PrototypeSmoke.cs')
    for source in ['Plugins/API/Foundation/Editor/BrushMesh.cs',
                   'Plugins/API/Foundation/Editor/BrushMeshUtility.cs',
                   'Plugins/API/Foundation/Runtime/SurfaceLayers.cs',
                   'Plugins/API/Foundation/Runtime/CSGOperationType.cs']:
        shutil.copyfile(ROOT / source, project / 'Assets/Recovery/Foundation' / Path(source).name)
    shutil.copyfile(library, project / 'Assets/Plugins/x86_64/librealtimecsg_recovered_geometry.so')
    (project / 'Packages/manifest.json').write_text(json.dumps({'dependencies': {}}, indent=2) + '\n')
    (project / 'ProjectSettings/ProjectVersion.txt').write_text('m_EditorVersion: ' + args.unity_version + '\n')
    executable = str(args.unity.resolve()) if args.unity else 'UNITY_EXECUTABLE'
    # The smoke uses asynchronous editor updates, so Unity must not receive -quit.
    command = [executable, '-batchmode', '-nographics', '-projectPath', str(project),
               '-executeMethod', 'RealtimeCSG.Recovery.PrototypeSmoke.BatchSmoke',
               '-logFile', str(project.parent / (project.name + '.log'))]
    print('Project: ' + str(project))
    print(shlex.join(command))


if __name__ == '__main__':
    main()
