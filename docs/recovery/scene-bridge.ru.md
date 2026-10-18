# Выдача сцены, C ABI и первый запуск в Unity Linux

Добавлен сквозной исследовательский путь от исходных выпуклых brushes и дерева
операций до UnityEngine.Mesh. Это **новый bridge v1**, а не замена существующих
GenerateTree/GenerateMeshDescriptions/GetGeneratedMesh и остальных экспортов
RealtimeCSG[1_559]. Основные C# bindings редактора не изменены и по-прежнему
обращаются к оригинальному модулю.

Файлы:

- `recovered_scene.h`: структуры и экспортируемый C ABI;
- `scene_build.cpp`: проверка и копирование исходных brushes, категоризация,
  подготовка, Optimize, gap repair и выдача;
- `scene_export.cpp`: атрибуты, материал-сортированные подмеши и происхождение;
- `scene_api.cpp`: immutable snapshots, handle registry и заполнение buffers;
- `native/managed/RecoveredScene.cs`: P/Invoke, pinning и IDisposable;
- `RecoveryPrimitives.cs`: небольшие явные fixtures для smoke/preview;
- `RecoveryUnityPreview.cs`: преобразование результата в Unity Mesh и явный preview.

## Подтверждённый Unity-запуск

В установленном Unity **2022.3.62f3, Linux x86_64** выполнен batch-тест в
изолированном проекте `/tmp/realtime-csg-recovery/unity-preview`.
Unity загрузил `librealtimecsg_recovered_geometry.so`, вызвал build/counts/copy
через C# и построил Mesh с позициями, нормалями, UV0, тангентами и подмешами.
Чтение mesh.vertices/triangles/normals/uv/tangents обратно из Unity прошло.

Исходный cube [-1,1]³ вычитается вертикальным прямоугольным резаком
[-.5,.5]×[-.5,.5]×[-2,2]. Результат:

```json
{"unity":"2022.3.62f3","vertices":96,"indices":96,"submeshes":4,"volume":6,"nativeAbi":1}
```

96 отдельных углов составляют **32 треугольника**. Сохранены Mesh asset,
четыре материала и scene. Первый результат —
`Assets/RecoveryOutput/RecoveryPreview.unity`, повторный —
`Assets/RecoveryOutput 1/RecoveryPreview.unity`. Повторный запуск выбирает новую
папку, не перезаписывая предыдущие assets. `RecoveryPreviewReport.json` указывает
папку последнего результата; log содержит `RECOVERY_UNITY_SMOKE_PASS`.

Это подтверждает загрузку библиотеки, P/Invoke и работу Unity Mesh API в
пакетном режиме без графического окна. Визуальный рендеринг и работа обычных
инструментов RealtimeCSG этим не проверялись. Исходный plugin/frontend в
этот изолированный проект не импортировался. В sandbox Unity сначала завершился
из-за доступа к своей служебной базе; успешные запуски выполнены с разрешённым
выходом из sandbox, без изменения кода или настроек лицензирования.

## Воспроизведение

```sh
cmake -S native -B /tmp/realtime-csg-recovery/build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /tmp/realtime-csg-recovery/build
ctest --test-dir /tmp/realtime-csg-recovery/build --output-on-failure
python3 tools/run_recovery_unity_smoke.py \
  --unity /home/angel/Unity/Hub/Editor/2022.3.62f3/Editor/Unity \
  --library /tmp/realtime-csg-recovery/build/librealtimecsg_recovered_geometry.so \
  --project /tmp/realtime-csg-recovery/unity-preview-new
```

Инструмент создаёт минимальный отдельный Unity-проект, копирует только bridge,
preview и .so, использует пустой manifest без внешних package dependencies.
Существующий непустой проект без его marker-файла отклоняется. `--prepare-only`
только подготавливает проект и печатает команду Unity. Сам Python runner с
запуском subprocess целиком пока отдельно не исполнялся: проверены его
prepare-only и напечатанная команда Unity. Команда Unity и обновлённый preview
успешно выполнены дважды.

В Unity доступен явный пункт меню
`Tools/RealtimeCSG Recovery/Create native subtract preview`.
Он создаёт отдельный GameObject, не переключая существующий RealtimeCSG.
Рабочая лицензия/доступ к локальному Unity требуются обычным образом.

Дополнительно [адаптер Foundation BrushMesh](foundation-bridge.ru.md) проверен
на исходном генераторе проекта: 27 сценариев в Unity с преобразованиями обоих
тел. Исходные типы и factory импортированы без editor startup; полный исходный
плагин этим не подключён.

## Вход bridge v1

Структуры имеют Pack=4, точные int32/uint16/byte и pointers. Проверены x86_64
sizes/offsets в C++ static_assert, Mono Marshal и самом Unity.

| Структура | Размер, x86_64 |
| --- | ---: |
| CategorizationNode / managed Node | 12 |
| SceneSurface | 52 |
| SceneBrushInput | 56 |
| SceneBuildInput | 60 |
| SceneVertex | 48 |
| SceneCounts | 44 |
| SceneOutputBuffers | 48 |
| SceneTriangleOrigin | 16 |
| SceneSubmesh | 12 |

SceneBrushInput содержит nodeIndex, реальный brushIndex, pointers/counts
vertices, halfedges, polygons и surfaces. Halfedge имеет vertexIndex/twinIndex;
полигон — непрерывный span рёбер и surfaceIndex. Это удобная проекция формы
открытого Foundation BrushMesh, но **не его полный ABI**: например, оригинальные
polygonID, smoothingGroup и SurfaceLayers ещё не передаются.

SceneSurface содержит outward unit plane (dot(n,p)-distance), две строки UV
по 4 float и непрозрачный signed materialId. Plane/vertices/UV rows находятся
в общем пространстве; преобразований из локальных пространств bridge не делает.
Материал — ID, не указатель на UnityEngine.Material. Preview сопоставляет ID
тестовым материалам; исходный Material registry пока не подключён.

Отдельный managed FoundationBridge уже преобразует локальные вершины,
плоскости и UV в общий model space, сохраняя polygonID/SurfaceLayers вне native
ABI. Это не меняет низкоуровневый контракт, описанный выше.

Входы копируются на native workspace. Проверяются surface/vertex/edge индексы,
reciprocal twins и их endpoints, полигонные spans/владение рёбрами, конечность,
единичность и выпуклые полупространства planes, принадлежность точек плоскостям
своих граней и положительная ориентация площади. Корректный замкнутый выпуклый
контур остаётся предусловием; универсальная починка невыпуклых/вырожденных
исходников не выполняется.

Параметры workspace задают maxBrushVertices/Edges/Polygons и gap budget.
MaxEdges кратен 4. По умолчанию managed build резервирует 2048/4096/512 записей
на brush и 128 gap cuts. Исчерпание возвращает ошибку, а исходные managed
массивы сохраняются: вся категоризация выполняется уже на owned native копиях.
Дерево должно укладываться в ограничения существующего компилятора категорий,
включая additive child в каждом branch; infinite/относительные transforms не
расширяются новым bridge.

## Экспорт атрибутов

Скрытое state 1 не выводится. State 3 разворачивает winding и знак нормалей;
при этом сохраняется принадлежность исходным brush/surface и материалу резака.
Численно вырожденные по новой scale-aware политике треугольники пропускаются;
порог и ограничение для очень тонких элементов описаны в
[отчёте Foundation](foundation-bridge.ru.md). Для каждого оставшегося видимого
треугольника выдаются три отдельных вершины. Сварка,
сглаживание и оптимизация индексов между нормальными/UV-швами пока не выполняются.

UV рассчитываются по явным строкам `dot(U.xyz,position)+U.w` и аналогично V.
Это соответствует модели UVMatrix открытого C# API, но воспроизведение всех
Legacy TexGen преобразований ещё не заявляется. Тангенты вычисляются из
градиентов UV и нормали, handedness учитывает зеркальную проекцию/обращение.
При вырожденной UV-проекции используется стабильный ортогональный tangent.
Это новая генерация, без заявления паритета с оригиналом или MikkTSpace.

Подмеши отсортированы по signed materialId и имеют непрерывные index ranges.
Внутри материала порядок исходных brushes/triangles сохраняется. Отдельный
SceneTriangleOrigin возвращает brushIndex, surfaceIndex, materialId, state,
flags для каждого выходного треугольника. Bounds вычисляются по видимым углам;
для пустого результата они нулевые. Unity adapter использует UInt32 индексы;
исходный протокол разбиения GeneratedMeshDescription на 64k chunks не реализован.

## Жизненный цикл и ошибки

1. `rcsg_recovered_scene_build` публикует immutable snapshot и uint64 handle.
2. `..._counts` возвращает необходимые размеры.
3. `..._copy` заполняет caller-owned arrays. До любых записей проверяются все
   ёмкости и перекрытие диапазонов: ошибка последнего buffer не меняет первые.
4. `..._release` удаляет handle; повторное использование отклоняется.

Registry защищён mutex и хранит shared snapshots. Начатый read удерживает
snapshot даже при конкурентном release; новые обращения после release получают
ошибку. Handles — счётчики, не pointers; память выделяется и освобождается
native стороной. Переданные при build buffers можно освободить после возврата.

Коды: 0 успех, -1 invalid/unsupported, -2 capacity/budget, -3 прочие исключения.
Исключения C++ не пересекают C ABI. Thread-local diagnostic доступен через
`rcsg_recovered_scene_last_error`. Неуспешный build не публикует handle.

Managed Scene закрепляет массивы только на время вызова, освобождает все pins
при исключении, защищает Read/Dispose одной блокировкой и имеет finalizer как
страховку освобождения. Явный using/Dispose предпочтителен. Перед выдачей
проверяются размеры/смещения и версия ABI; 32-bit managed process отклоняется.

## Проверки

- Все прежние 740 CSG-результатов теперь дополнительно экспортируются в единый
  material-grouped scene buffer. Объём/площадь сравниваются с независимым
  эталоном; проверяются конечность UV, единичность и ортогональность tangents.
- 24-й native CTest: UV/offset, sorted materials, hidden output, reversed winding,
  flipped normals/handedness, mirrored/degenerate UV, bounds, полный build трёх
  булевых операций и cavity с происхождением поверхностей резака.
- 25-й: C ABI counts/copy, неизменность buffers при capacity/alias errors,
  snapshot после изменения источника, конкурентный counts/release, stale handles,
  версия ABI, исчерпание clip workspace и некорректный halfedge.
- 26-й, если доступны mcs/mono: настоящие P/Invoke из C#, volumes 12/4/4,
  UV/tangents/material ranges, cavity volume 7, освобождение, пустое пересечение
  и получение native diagnostic. Это не Unity mock.

Обычная сборка: все **26 CTest** проходят. ASan/UBSan проверяют все **25 native
CTest**; managed hosting намеренно не запускается с ASan library. Реальный Unity
smoke выполнен отдельно и описан выше. Ни ошибку загрузки .so, ни успех
GetGeneratedMesh старого frontend этот bridge не маскирует.

## Следующая интеграция

Первый сквозной путь в Unity уже подтверждён. Чтобы он стал рабочим режимом
исходного редактора, нужны отображение существующих CSGTree/BrushMesh IDs и
жизненного цикла, адаптация Foundation/Legacy surfaces/materials/transforms,
dirty/undo/domain reload, mesh queries, selection/контуры и поддержка
расширенных сцен. Полной совместимости RealtimeCSG этим не объявляется.
