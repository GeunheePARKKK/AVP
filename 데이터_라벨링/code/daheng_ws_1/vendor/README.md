# vendor/gxipy

Copied from:

    C:\Program Files\Daheng Imaging\GalaxySDK\Development\Samples\Python\gxipy

The Galaxy SDK ships its Python API as a sample source folder, not an installable
package, so it is vendored here rather than added to `sys.path` from Program Files.

## Local patch

`DeviceManager.py` — the SDK imports `from numpy.compat import long`, and
`numpy.compat` was removed in numpy 2.0 (this machine has numpy 2.4.6), so the
import is now guarded:

```python
try:
    from numpy.compat import long
except ImportError:
    long = int
```

`long` is only referenced in the `sys.version_info.major < 2` branch, which never
runs on Python 3, so the fallback value is irrelevant to behaviour.

Re-apply this patch if you re-copy the folder after an SDK update.
