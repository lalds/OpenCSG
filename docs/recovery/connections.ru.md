# FindConnections: точное поведение macOS-версии

`native/recovered/connections.cpp` восстанавливает
`TriangleMesh::FindConnections` (`0x35840..0x35de6`). Интерфейс —
`native/include/realtimecsg/recovered_connections.h`;
C-функция — `rcsg_recovered_find_connections`.

Источник: Mach-O x86_64 RealtimeCSG 1.559, SHA-256
`26789e84b0c11376582662bf1d53ed21a65c68d57449efb607070b266cb61a7d`.
Эквивалентность этой функции Windows DLL ещё не проверена.

## Неожиданная адресация manager

Исходная функция получает `this` TriangleMesh, brushIndex и vector<BrushEdge>.
Вызывающий `UpdateTreeMeshes` передаёт mesh и ID одного браша, затем его
BrushEdge-вектор: инструкции `0x184bd..0x184cf`.

FindConnections собирает уникальные brush IDs из surfaceRefs текущей группы.
Однако при поиске кандидатов manager индексируется **аргументом brushIndex**,
а не очередным ID из этого множества. Это видно по `r12d`:
аргумент сохраняется по `0x3585b`, проверки/индексация используют его по
`0x35a31..0x35a7f`. Ключ множества читается в `r14d` по `0x35ab5` и используется
только при проверке surfaceRefs кандидата по `0x35bdd`.
Добавляемый brush ID также равен аргументу brushIndex (`0x35c20`).

Это подтверждено исполнением неизменённого оригинала, а не только чтением
регистров. Минимальный пример:

```text
brushIndex = 0
local group 0: surfaces (3,4), line X, surfaceRefs [(1,3)]
manager[0] contains local group 0
manager[1] group 0: same line, surfaceRefs [(0,3)]
original result for local group 0: [(0,0)]
```

В этом примере создаётся ссылка на собственную группу.
Из одного macOS-бинарника нельзя заключить, что это одинаковая ошибка во всех
версиях плагина. Восстановленная функция сохраняет наблюдаемую адресацию;
исправленный поиск по foreign ID здесь не добавлялся.

```sh
python3 -B tools/compare_find_connections.py --diagnostic
```

## Остальные восстановленные правила

Для каждой локальной группы в исходном порядке:

1. Прочитать пару локальных surface indices.
2. Получить уникальные brush IDs из surfaceRefs и отсортировать их как signed int.
3. Для каждого ID проверить manager-узел по аргументу brushIndex: диапазон,
   metadata.kind == 3, наличие node, совпадение node.index.
4. Обойти все группы выбранного узла в порядке их индексов.
5. Проверить направления и расстояние между прямыми.
6. Кандидат подходит, если среди его surfaceRefs есть пара с текущим ключом
   множества и surface index, равным одному из двух local surface indices.
7. Добавить `(brushIndex,candidateGroupIndex)` в локальный connection vector.

Пара surface indices самого кандидата не сравнивается с локальной парой.
Одного подходящего surfaceRef достаточно для добавления кандидата один раз
за текущий ключ множества. Несколько разных ключей могут добавить его повторно.
SurfaceRefs с отрицательным ID также участвуют: этот ID не используется для
индексации manager на данном этапе.

Направления проходят при `abs(dot(candidateDirection,localDirection)) >
0.9999899864196777f`; unordered сравнение с NaN также проходит исходный
`ucomiss; jae`. Затем точка кандидата проецируется на локальную прямую:

```text
t = dot(candidateOrigin-localOrigin, localDirection)
projected = localOrigin + localDirection*t
accept if |projected-candidateOrigin|² < 1.0000000116860974e-7f
```

Операции float, порядок суммирования `(x+y)+z`. Направления здесь не
нормализуются. Если расстояние получается NaN, кандидат отклоняется.

Существующие connections **сохраняются**. Новые результаты дописываются
без дедупликации. Повторный вызов может добавить ту же последовательность снова.
Вызов reserve в оригинале учитывает текущие connections и число surfaceRefs,
но не очищает вектор.

Если итоговый connection vector пуст, для всех triangle handles этой группы
записывается `triangle.edgeSource[corner] = -1`. Сами handles и их список,
visibility, brushEdges, twin-связи и остальные поля треугольников не меняются.
Непустая исходная connection list предотвращает очистку даже при отсутствии
новых кандидатов. Source handles могут повторяться, порядок групп сохраняется.

## Переносимый интерфейс

`BrushConnectionNode` содержит группы, их число, nodeIndex и nodeKind.
Отсутствующий узел представляется несовпадающим/отрицательным nodeIndex.
Меш/planes/transform manager-узла FindConnections не читает.
Выходные `BrushEdgeConnectionList` и входные triangle-handle lists передаются
отдельными массивами, согласованными с массивом BrushEdgeGroupView.

Буферы имеют фиксированную ёмкость. При её исчерпании возвращается -2,
при неверных views/handles — -1. Ошибка после предыдущих добавлений или групп
может оставить выполненную часть работы. Некорректные handles на этапе очистки
проверяются до записей этой группы; оригинал разыменовывает их без проверки.

## Сравнение на оригинальном коде

`tools/compare_find_connections.py` загружает неизменённый FindConnections,
его vector helpers, оригинальный std::tree insert/balance/destroy.
Диапазоны тела функций и зависимостей берутся из локального аудита символов.
SHA-256 проверяется перед загрузкой, код RX и данные RW; Unity и системные
инициализаторы Mach-O не выполняются. Аллокаторы привязаны к Linux libc.

При seed 1559:

| Показатель | Результат |
| --- | --- |
| Наборы | 5 000 |
| Вызовы оригинала | 10 000 |
| Добавленные connections | 24 228 |
| Итоговые очищенные triangle edgeSource поля | 25 552 |
| Расхождения connections и полных triangle buffers | 0 |

Каждый набор вызывается дважды, что проверяет сохранение/дописывание списков.
Есть пустые группы, signed IDs, повторные surfaceRefs, существующие connections,
разные manager IDs и состояния узлов, несколько кандидатов, порядок обхода,
положительные/отрицательные направления, соседние float у порога и NaN.
Детерминированные примеры дополняют случайные входы: сортировка ключей -1,2,5,
кандидат с другой парой собственных surfaces, пустая группа с сохранённой связью.
Проверяется неизменность source handles, surfaceRefs, line records,
нелокальных group records и connections.

Connection vectors имеют запас ёмкости, поэтому native reserve не требует
их перевыделения. Оригинальное множество выделяет и освобождает свои узлы
обычным образом; отсутствие утечек этих аллокаций проверяется обёртками.

Двенадцатый CTest проверяет эталонный порядок, адресацию по brushIndex,
добавление повторов, очистку/сохранение edgeSource, manager filters,
расстояние и точный порог направлений, ёмкость и неверные handles.
Все 12 CTest проходят в обычной сборке и с ASan/UBSan
(`ASAN_OPTIONS=detect_leaks=0`).

```sh
cmake --build /tmp/realtime-csg-recovery/build
ctest --test-dir /tmp/realtime-csg-recovery/build --output-on-failure
python3 -B tools/compare_find_connections.py --library /tmp/realtime-csg-recovery/build/librealtimecsg_recovered_geometry.so --cases 5000
```

## Ограничение интеграции

Полная цепочка FindBrushEdges → Create/triangle lists → FindConnections →
FixGaps пока не объявляется рабочей. Triangle lists уже заполняются
[перенесённым CleanUpBrushEdges](brush-edge-cleanup.ru.md) для согласованных
групп, включая конфликтующие пары. Добавлен и
[MergeBrushIndices для массивов](merge-brush-indices.ru.md) и
[Create](mesh-creation.ru.md). Найденная адресация
может давать self references. Ранее написанный
FixGaps-адаптер требует независимых local/foreign buffers и такие ссылки
отклоняет. Простой вызов восстановленных функций подряд этого не исправляет.
Нужны проверка Windows-поведения и решение на уровне общего конвейера;
это отдельная задача от точного воспроизведения данной macOS-функции.

В исследовательском конвейере это решено отдельным [foreign resolver](convex-gap-repair.ru.md),
который смотрит по реальному foreign ID и требует обратных surface refs.
Он выполняет новую политику; эта восстановленная функция и её C ABI не менялись.
