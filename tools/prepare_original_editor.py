#!/usr/bin/env python3
"""Prepare the original C# RealtimeCSG editor with the open Linux native backend."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import os
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--unity', type=Path)
    args = parser.parse_args()
    project = args.project.resolve()
    marker = project / '.opencsg-original-editor'
    if project.exists() and any(project.iterdir()) and not marker.is_file():
        parser.error('Refusing to modify an unrelated project')
    if (project / 'Temp/UnityLockfile').exists():
        parser.error('Close Unity before updating this project (native plugins require an editor restart)')
    libraries = [args.build / 'libRealtimeCSG[1_559].so', args.build / 'librealtimecsg_recovered_geometry.so']
    for library in libraries:
        if not library.is_file():
            parser.error('Missing library: ' + str(library))
    assets = project / 'Assets/RealtimeCSG'
    assets.mkdir(parents=True, exist_ok=True)
    marker.write_text('Original C# frontend with open experimental Linux backend\n')
    shutil.copytree(ROOT / 'Plugins', assets / 'Plugins', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('*.dll', '*.dll.meta', '*.bundle', '*.bundle.meta'))
    for name in ['RealtimeCSG.asmdef', 'RealtimeCSG.asmdef.meta', 'Icons.zip', 'Icons.zip.meta']:
        if (ROOT / name).is_file():
            shutil.copyfile(ROOT / name, assets / name)
    for library in libraries:
        # Never truncate a shared library that could still be mapped by a process.
        destination = assets / 'Plugins/Editor/External' / library.name
        with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as temporary:
            temporary_path = Path(temporary.name)
        try:
            shutil.copyfile(library, temporary_path)
            os.replace(temporary_path, destination)
        finally:
            temporary_path.unlink(missing_ok=True)
    shutil.copyfile(ROOT / 'native/tests/unity_original_editor_smoke.cs', assets / 'Plugins/Editor/OriginalEditorSmoke.cs')
    for folder in ['Packages', 'ProjectSettings']:
        (project / folder).mkdir(exist_ok=True)
    modules = ['imgui', 'ui', 'uielements', 'physics', 'physics2d', 'animation', 'audio', 'imageconversion', 'jsonserialize', 'terrain', 'terrainphysics']
    (project / 'Packages/manifest.json').write_text(json.dumps({'dependencies': {
        'com.unity.modules.' + module: '1.0.0' for module in modules
    }}, indent=2) + '\n')
    (project / 'ProjectSettings/ProjectVersion.txt').write_text('m_EditorVersion: 2022.3.62f3\n')
    command = [str(args.unity.resolve()) if args.unity else 'UNITY_EXECUTABLE', '-batchmode', '-nographics',
               '-projectPath', str(project), '-executeMethod', 'RealtimeCSG.Recovery.OriginalEditorSmoke.BatchSmoke',
               '-logFile', str(project.parent / 'original-editor.log')]
    print(shlex.join(command))


if __name__ == '__main__':
    main()
