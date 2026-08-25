## Known issue

The building YAML currently refers to:

```text
../../../maps/robot2_map.png
```

This background image is missing from the migrated environment.

Current RMF simulation works because the generated assets
(nav_graphs, world, models) are already available.

If the building map needs to be edited again with Traffic Editor,
the original background image should be restored or the YAML updated.

## Known issue: Traffic Editor background image

The migrated building YAML currently refers to:

```text
../../../maps/robot2_map.png
```

The referenced image is missing from the migrated environment.

The generated RMF assets remain available:

navigation graph
Gazebo world
generated models

Therefore, runtime use is possible, but the original background image should be
restored before editing or regenerating the building map with Traffic Editor.
Do not substitute another map image unless its identity and coordinate
alignment are verified.