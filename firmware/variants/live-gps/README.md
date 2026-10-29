# Live GPS experimental variant

This branch preserves the supplied alternate RIDeGUARD sketch as an experimental firmware variant.

The variant differs from the firmware currently published on `main`:

- demo/reference coordinates start near latitude 12.844170 and longitude 80.151240;
- no-fix fallback coordinates and altitude are randomized within small bounds every 2 seconds;
- the dashboard label may change from `DEMO DATA` to `LIVE`, even though fallback data is synthetic;
- satellite count may randomly change while no real GPS fix is available.

These differences have not been hardware-verified. The random fallback coordinates must not be presented as a genuine live location. Before any production use, replace demo fallback behavior with a clearly labeled no-fix state or separate synthetic data mode and verify real NEO-6M fixes outdoors.
