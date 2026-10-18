# Адаптер исходного Foundation BrushMesh

2026-10-18. Добавлен `native/managed/RecoveryFoundationBridge.cs`: существующий
`RealtimeCSG.Foundation.BrushMesh` преобразуется в вход нового scene bridge.
Это подключение формата данных, не подмена оригинальных Foundation exports
и не включение всего редактора на Linux.

## Подтверждённая точка подключения

`InternalCSGModelManager.Registration.cs`, SetBrushMesh, сначала вызывает
`Legacy.BrushFactory.GenerateFromControlMesh`, затем передаёт Foundation BrushMesh
в External.CreateBrushMesh/UpdateBrushMesh. Новый адаптер принимает именно этот
тип. Таблица External не покрывает прямые Foundation DllImport; её частичная
подмена сама по себе не обеспечит работу исходного плагина.

## Преобразования и атрибуты

`FoundationBridge.Convert(source, modelFromBrush, worldFromModel, nodeIndex,
brushIndex)` возвращает FoundationBrush: собственную геометрию и копии описаний
исходных полигонов. Источник не меняется. SourcePolygon(surfaceIndex) возвращает
сохранённые polygonID, SurfaceDescription и SurfaceLayers. Выходной
TriangleOrigin.surfaceIndex индексирует эти записи, а brushIndex выбирается
вызывающей стороной как уникальный неотрицательный ID нового ядра.

- Вершины переводятся в пространство модели.
- Outward planes вычисляются из обхода исходных граней и преобразуются
  обратной транспонированной матрицей, с нормализацией.
- При отрицательном determinant меняются порядок рёбер и их конечные вершины;
  twin indices перенумеровываются согласованно. Один разворот списка вершин
  недостаточен при endpoint-представлении halfedge.
- Для локальных UV строки умножаются на brushFromModel; для флага
  TextureIsInWorldSpace — на worldFromModel. Это явная интерпретация контракта
  Foundation UVMatrix; паритет всех Legacy WorldSpaceTexture путей с закрытым
  модулем ещё не проверен.
- layerParameter1 становится signed materialId. Остальные параметры и flags
  сохранены в managed metadata, но не превращаются автоматически в отдельные
  collider/shadow meshes. Сглаживание (smoothingGroup != 0) явно отклоняется;
  неизвестные surface flags также отклоняются.
- Поддержаны проверенные невырожденные affine transforms, в том числе shear.
  Projective/nonfinite/singular и численно нестабильные обратные матрицы
  отклоняются. Последующая native проверка topology/convexity остаётся в силе.

## Воспроизводимая проверка в Unity

В Unity 2022.3.62f3 Linux x86_64 прошли **27 сценариев**. В изолированный проект
копируются без изменений три исходных файла проекта: BrushMesh.cs,
BrushMeshUtility.cs, SurfaceLayers.cs. Исходный editor startup и закрытая
библиотека не импортируются. Кубы создаёт настоящий BrushMeshUtility.CreateCube,
а не Recovery.Primitives; от последнего используется только явная таблица дерева.

Проверены восемь отдельных преобразований куба (translation, rotation,
nonuniform scale, reflection, combinations, shear), три булевы операции с
повёрнутым отражённым резаком, ещё девять операций после общего affine
преобразования обоих тел и семь случаев отказа. Объёмы сравниваются с
независимыми 8*abs(det), 10/6/2*abs(det). Дополнительно проверены UV по исходным
строкам и обратным матрицам, нормали/тангенты, положительная ориентация каждого
треугольника, IDs/layers и неизменность исходных массивов/metadata.

Расширенная проверка выявила почти коллинеарный треугольник после общей
отражённой трансформации. Добавлена **новая политика экспорта**, без изменения
восстановленных функций ядра: треугольники с модулем cross product не больше
`8 * float_epsilon * max_abs_coordinate * max_edge_length` не выводятся.
Расчёт cross выполняется в double; индексы и атрибуты проверяются до фильтра.
Это консервативный порог, учитывающий масштаб и величину координат, а не
доказательство точности всей геометрии. Очень тонкие элементы, особенно далеко
от начала координат, могут попасть под фильтр; произвольные масштабы этим не
сертифицированы. В native scene export CTest добавлен регрессионный пример,
проверка сохранения различимого треугольника при масштабировании и отказа при
некорректном индексе.

Успешный Unity report:

```json
{"unity":"2022.3.62f3","cases":27,"volume":5.99999993801048,"nativeAbi":1,"source":"original Foundation BrushMeshUtility","assetDirectory":"Assets/FoundationOutput 1"}
```

Проект: `/tmp/realtime-csg-recovery/unity-foundation`.
Сцена: `Assets/FoundationOutput 1/FoundationPreview.unity`.
Report: `RecoveryFoundationReport.json`; log:
`/tmp/realtime-csg-recovery/unity-foundation.log`.
Это batch readback, не проверка визуального рендеринга или инструментов редактора.

```sh
python3 tools/run_recovery_unity_smoke.py --foundation --prepare-only \
  --library /tmp/realtime-csg-recovery/build/librealtimecsg_recovered_geometry.so \
  --project /tmp/realtime-csg-recovery/unity-foundation-new
```

Подставить установленный Unity вместо UNITY_EXECUTABLE в напечатанной команде.
prepare-only и прямой Unity запуск проверены. Полный Python subprocess runner
по-прежнему отдельно не проверялся.

## Точка продолжения

1. Подключить адаптер к registry исходного дерева: стабильные ID, порядок детей,
   операции, создание/удаление/перенос brushes и матрицы относительно модели.
2. Установить поведение неаддитивных вложенных branches и расширить ограничения
   существующего компилятора категорий по оригинальному коду.
3. Выдавать render/collider/shadow meshes по SurfaceLayers, сопоставлять реальные
   Unity Material и сохранить polygonID для выбора поверхностей.
4. Подключить dirty/Undo/domain reload и пересборку мешей, затем raycast/контуры.
5. Проверять реальные сцены и оставшиеся вырождения; сохранять сравнения с
   оригинальными функциями, не объявляя полную совместимость по smoke-тестам.

Работа сохранена в файлах репозитория; git commit не создавался. Каталоги docs,
native и tools пока целиком untracked — это нужно учесть перед переносом работы.
