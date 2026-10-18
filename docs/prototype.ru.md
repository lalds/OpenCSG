# Первый редактируемый прототип Unity

Прототип использует открытое ядро и отдельные компоненты RecoveryModel /
RecoveryBrush. Проверяемая конфигурация — Unity 2022.3.62f3, Linux x86_64,
Built-in Render Pipeline. Исходный менеджер RealtimeCSG и его закрытый модуль
для этого режима не нужны.

## Запуск готового проекта

Проект находится в `artifacts/UnityPrototype`. Добавить эту папку в Unity Hub
и открыть в Unity 2022.3 LTS. Затем открыть сцену:

`Assets/PrototypeOutput 1/EditablePrototype.unity`

В сцене есть RecoveryModel, дочерние Solid и Cutter, камера и направленный свет.
Выбрать Solid/Cutter в Hierarchy и менять Transform или Size в Inspector.
Результат пересчитывается автоматически. Undo/Redo поддержаны. Поле Material
задаёт настоящий материал поверхности; внутренние поверхности вычитания получают
материал резака. Brushes видны в Scene view как цветные wireframe gizmos.

Чтобы создать другую модель:

1. `Tools > RealtimeCSG Recovery > Create editable CSG prototype`.
2. Выбрать модель или её brush; добавлять brushes через `Add additive brush`,
   `Add subtractive brush`, `Add intersecting brush` либо кнопки Inspector.
3. Вычисление идёт в порядке Hierarchy, включая обход обычных родительских
   Transform. Brushes под другой RecoveryModel принадлежат другой модели.
4. В Inspector модели доступны Rebuild и Export mesh asset. Экспорт сохраняет
   отдельную копию Unity Mesh; её можно использовать в обычном MeshFilter.

`artifacts/OpenCSG-Prototype.unitypackage` содержит компоненты, инспекторы,
исходные Foundation data types и Linux .so. Импортировать его в **отдельный
чистый Unity-проект**. Пакет не следует импортировать рядом с полным исходным
RealtimeCSG: в обоих есть одинаковые Foundation types. Сама демонстрационная
сцена и её материалы находятся в готовом проекте, в package они не входят.
Компоненты работают в редакторе; для player build нужно экспортировать и
использовать статический mesh. Сгенерированный preview скрыт и не сохраняется
в build. Настройки .so в готовом пакете включают только Linux x86_64 Editor.

## Что реализовано

- Box brushes с размером, материалом и Additive/Subtractive/Intersecting.
- Поворот, перенос, неравномерный и отрицательный масштаб через FoundationBridge.
- Автоматическая пересборка при изменениях размеров, операций, порядка,
  Transform, включения и материалов; Undo/Redo.
- Несколько независимых моделей; пустая модель очищает output.
- Проверка вводов: при ошибке Inspector показывает сообщение, последний
  корректный mesh остаётся видимым. Восстановление ввода запускает пересборку.
- Собственный transient Unity Mesh, материалы по submeshes, normals/UV/tangents,
  экспорт копии mesh asset и повторная генерация при открытии сцены.
- Очистка сгенерированных объектов; пользовательский объект с совпадающим
  названием output не присваивается модели и не удаляется.

Компоненты и UI находятся в `native/managed/RecoveryModel.cs`, RecoveryBrush.cs,
RecoveryPrototypeEditor.cs. Компоненты компилируются вне Editor assembly,
инспекторы — внутри неё; runtime-to-editor ссылки отсутствуют. Все эти компоненты
ограничены UNITY_EDITOR.

## Проверка

`native/tests/unity_prototype_smoke.cs` запускает настоящий Unity Editor update
loop, создаёт модель через пункт меню и проверяет 34 условия: объём, отсутствие
постоянной пересборки в покое, автоматическое обновление после движения,
Undo/Redo, union/intersect/subtract, отключение brush, отражение/масштаб,
сохранение последнего корректного mesh при нулевом размере, восстановление,
материалы резака, вращение, Transform модели, очистку и сохранение сцены.

Успешный batch-запуск на Unity 2022.3.62f3: 34 проверки, 15 успешных сборок,
итоговый объём `5.9999999801317614` при ожидаемых 6. Сообщение об ошибочном
нулевом размере в логе намеренное: это проверка отказа и сохранения результата.
Отдельный `ReloadSmoke` повторно открывает сохранённую сцену в новом процессе
Unity и проверяет исходные brushes, объём и отсутствие дублированного output.
Отчёты находятся в корне подготовленного проекта.

Нативная сборка с текущего расположения репозитория и все 26 CTest проходят.
Графическое окно и фактический рендер batch-проверками не проверяются.

## Воспроизведение из исходников

```sh
cmake -S native -B /tmp/opencsg-prototype-build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /tmp/opencsg-prototype-build -j 4
ctest --test-dir /tmp/opencsg-prototype-build --output-on-failure
python3 tools/prepare_recovery_prototype.py \
  --project /tmp/opencsg-prototype \
  --library /tmp/opencsg-prototype-build/librealtimecsg_recovered_geometry.so \
  --unity /home/angel/Unity/Hub/Editor/2022.3.62f3/Editor/Unity
```

Последняя команда создаёт изолированный Unity-проект и печатает команду batch
проверки. Запускать её без `-quit`: smoke асинхронный и сам завершает Unity.
После успеха получается `OpenCSG-Prototype.unitypackage`. Для проверки открытия
сцены в другом процессе использовать executeMethod
`RealtimeCSG.Recovery.PrototypeSmoke.ReloadSmoke`.

Следующий этап уже добавлен: [исходный C# редактор](original-editor.ru.md)
подключается к открытому ABI-backend через `RCSG_BUILD_UNITY_COMPAT_PLUGIN=ON`.
Этот отдельный box-прототип продолжает использовать исследовательский bridge.

## Границы прототипа

Это самостоятельный редактор box brushes. Здесь ещё нет исходных инструментов
вершин/граней RealtimeCSG, surface picking, контуров после CSG, вложенных
operation groups, сглаживания, collider/shadow queries и инкрементального ядра.
Модель пересобирается целиком; режим рассчитан на небольшие сцены. Нужен хотя бы
один Additive brush; операции до первого Additive обрабатываются по правилам
текущего компилятора категорий. Вырождения и произвольные тонкие элементы
остаются ограничением геометрического ядра; новый фильтр почти нулевых площадей
описан в foundation-bridge.ru.md. Другие версии Unity и другие ОС здесь не
проверены. Демо использует Standard materials; для другого render pipeline
следует назначить материалы подходящего shader.
